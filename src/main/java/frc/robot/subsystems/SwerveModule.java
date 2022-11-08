// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.subsystems.SwerveDrive;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class SwerveModule extends SubsystemBase {
  private static final double kWheelRadius = 0.1; // TODO: wheel radius in metres
  private static final int kEncoderResolution = 4096; // TODO: actual encoder resolution

  private final CANSparkMax m_driveMotor;
  private final CANSparkMax m_rotationMotor;
  private final int m_rotationEncoder; // TODO

  /** Creates a new SwerveModule. */
  public SwerveModule(
      int driveMotorID,
      int rotationMotorID,
      int rotationEncoderPort) {
    m_driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
    m_rotationMotor = new CANSparkMax(rotationMotorID, MotorType.kBrushless);
    m_rotationEncoder = rotationEncoderPort;
  }

  /**
   * Returns the current state of the module.
   * 
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(m_driveEncoder.getRate(), new Rotation2d(m_rotationEncoder.get()));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState state =
        SwerveModuleState.optimize(desiredState, new Rotation2d(m_rotationEncoder.get()));
    
    // Calculate the drive output from the drive PID controller.
    final double driveOutput =
        m_drivePIDController.calculate(m_driveEncoder.getRate(), state.speedMetersPerSecond);

    final double driveFeedForward = m_driveFeedForward.calculate(state.speedMetersPerSecond);
    
    // Calculate the rotation motor output from the rotation PID controller.
    final double rotationOutput =
        m_rotationPIDController.calculate(m_rotationEncoder.get(), state.angle.getRadians());

    final double rotationFeedForward =
        m_rotationFeedForward.calculate(m_rotationPIDController.getSetpoint().velocity);

    m_driveMotor.setVoltage(driveOutput + driveFeedForward);
    m_rotationMotor.setVoltage(rotationOutput + rotationFeedForward);
  }
}
