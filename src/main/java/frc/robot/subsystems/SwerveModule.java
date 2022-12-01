// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.Swerve.*;
import static frc.robot.Constants.Swerve.Gains.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

/**
 * Represents a single swerve drive module.
 */
public class SwerveModule {
  private final CANSparkMax m_driveMotor;
  private final CANSparkMax m_rotationMotor;

  private final RelativeEncoder m_driveEncoder;
  private final RelativeEncoder m_rotationEncoder;

  private final PIDController m_drivePIDController =
      new PIDController(drivePID_kP, drivePID_kI, drivePID_kD);
  private final ProfiledPIDController m_rotationPIDController =
  new ProfiledPIDController(
      rotationPID_kP,
      rotationPID_kI,
      rotationPID_kD,
      new TrapezoidProfile.Constraints(
          MAX_ANGULAR_ACCELERATION, MAX_ANGULAR_VELOCITY));

  private final SimpleMotorFeedforward m_driveFeedforward =
      new SimpleMotorFeedforward(driveFF_kS, driveFF_kV);
  private final SimpleMotorFeedforward m_rotationFeedforward =
      new SimpleMotorFeedforward(rotationFF_kS, rotationFF_kV);

  /** Creates a new SwerveModule. */
  public SwerveModule(int driveMotorID, int rotationMotorID) {
    m_driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
    m_rotationMotor = new CANSparkMax(rotationMotorID, MotorType.kBrushless);

    m_driveEncoder = m_driveMotor.getEncoder();
    m_driveEncoder.setPositionConversionFactor(DRIVE_POSITION_CONVERSION);
    m_driveEncoder.setVelocityConversionFactor(DRIVE_VELOCITY_CONVERSION);
    
    m_rotationEncoder = m_rotationMotor.getEncoder();
    m_rotationEncoder.setPositionConversionFactor(ROTATION_POSITION_CONVERSION);

    /* Problem description:
     * The Spark Max encoder can be any negative or positive number, it is not confined to a range.
     * The min and max in enableContinousInput will act as the same point.
     * My guess is that WPILib assumed the swerve module would never rotate more than 180 degrees.
     * This assumption is correct provided that SwerveModuleState.optimize() is used (never rotates more than 90 degrees).
     * However, if someone spins the module or the robot is bumped, it could go out of range.
     * What happens if you exceed the minimum or maximum values? Bad or acceptable behaviour?
     */
    /* Testing (Attempt when swerve modules are completed):
     * 1. Disable the robot with the swerve modules in a valid position (-180 to 180 degrees).
     * 2. Manually move one swerve a full rotation (360 degrees in either direction).
     * 4. (Make sure the module ends up in the same position it started!)
     * 5. Confirm the position with Shuffleboard.
     * 6. Re-enable the robot. Does the swerve module operate normally, or does it randomly change directions?
     * 7. If it behaves, do nothing.
     * 8. If it doesn't, divide m_rotationEncoder.getPosition() by Math.PI in getState() to limit it to 1 radian in either direction.
     */
    m_rotationPIDController.enableContinuousInput(Math.PI, Math.PI);
    // equivalent to -180 degrees and 180 degrees
  }

  /**
   * Returns the current state of the module.
   * 
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(
      m_driveEncoder.getVelocity(),
      new Rotation2d(m_rotationEncoder.getPosition())
    );
  }

  /**
   * Reset the rotation and drive encoders. DOES NOT HOME THE MODULE!
   * This should be used to reset the encoder position after manually homing
   * the module.
   */
  public void resetEncoders() {
    m_driveEncoder.setPosition(0);
    m_rotationEncoder.setPosition(0);
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState state =
        SwerveModuleState.optimize(desiredState, new Rotation2d(m_rotationEncoder.getPosition()));
    
    // Calculate the drive output from the drive PID controller.
    final double driveOutput =
        m_drivePIDController.calculate(m_driveEncoder.getVelocity(), state.speedMetersPerSecond);

    final double driveFeedforward = m_driveFeedforward.calculate(state.speedMetersPerSecond);
    
    // Calculate the rotation motor output from the rotation PID controller.
    final double rotationOutput =
        m_rotationPIDController.calculate(m_rotationEncoder.getPosition(), state.angle.getDegrees());

    final double rotationFeedforward =
        m_rotationFeedforward.calculate(m_rotationPIDController.getSetpoint().velocity);

    m_driveMotor.setVoltage(driveOutput + driveFeedforward);
    m_rotationMotor.setVoltage(rotationOutput + rotationFeedforward);
  }
}
