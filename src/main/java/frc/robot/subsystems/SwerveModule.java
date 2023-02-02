// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.Swerve.*;
import static frc.robot.Constants.Swerve.Gains.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
//import edu.wpi.first.util.sendable.Sendable;
//import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Represents a single swerve drive module.
 */
public class SwerveModule {//implements Sendable {
  private final CANSparkMax m_driveMotor;
  private final CANSparkMax m_rotationMotor;

  private final RelativeEncoder m_driveEncoder;
  private final RelativeEncoder m_rotationEncoder;

  private final PIDController m_drivePIDController =
      new PIDController(drivePID_kP, drivePID_kI, drivePID_kD);

  private final SimpleMotorFeedforward m_driveFeedforward =
      new SimpleMotorFeedforward(driveFF_kS, driveFF_kV, driveFF_kA);

  private final SparkMaxPIDController m_rotationPIDController;

  private final String m_name;

  /** Creates a new SwerveModule. */
  public SwerveModule(String name, int driveMotorID, int rotationMotorID) {
    m_driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
    m_rotationMotor = new CANSparkMax(rotationMotorID, MotorType.kBrushless);

    m_driveEncoder = m_driveMotor.getEncoder();
    m_driveEncoder.setPositionConversionFactor(DRIVE_POSITION_CONVERSION);
    m_driveEncoder.setVelocityConversionFactor(DRIVE_VELOCITY_CONVERSION);
    
    m_rotationEncoder = m_rotationMotor.getEncoder();
    m_rotationEncoder.setPositionConversionFactor(ROTATION_POSITION_CONVERSION);
    m_rotationEncoder.setVelocityConversionFactor(ROTATION_VELOCITY_CONVERSION);

    m_rotationPIDController = m_rotationMotor.getPIDController();
    configureSparkMaxPID();

    m_name = name;

    // SparkMaxPIDController and SimpleMotorFeedForward are not sent as they do not implement Sendable.
    SendableRegistry.setName(m_drivePIDController, m_name, "Drive PID Controller");

    /* Send these values in the constructor so they appear in NetworkTables
     * before setDesiredState() is called for the first time. */
    SmartDashboard.putNumber(m_name + " Drive Setpoint (m/s)", 0);
    SmartDashboard.putNumber(m_name + " Angle Setpoint (rad)", 0);
    SmartDashboard.putNumber(m_name + " Drive Voltage", 0);
  }

  /** Update data being sent and recieved from NetworkTables. */
  public void updateNetworkTables() {
    SmartDashboard.putNumber(m_name + " Angle (rad)", m_rotationEncoder.getPosition());
    SmartDashboard.putNumber(m_name + " !!Angular Velocity (radps)", m_rotationEncoder.getVelocity());
    SmartDashboard.putNumber(m_name + " Distance Travelled (m)", m_driveEncoder.getPosition());
    SmartDashboard.putNumber(m_name + " Velocity (mps)", m_driveEncoder.getVelocity());
    SmartDashboard.putNumber(m_name + " Drive Motor Temp (C°)", m_driveMotor.getMotorTemperature());
    SmartDashboard.putNumber(m_name + " Rotation Motor Temp (C°)", m_rotationMotor.getMotorTemperature());
  }

  /** Configure data being sent and recieved from NetworkTables. */
  // @Override
  // public void initSendable(SendableBuilder builder) {
  //   builder.addDoubleProperty("Distance Travelled (m)", m_driveEncoder::getPosition, null);
  //   builder.addDoubleProperty("Velocity (m/s)", m_driveEncoder::getVelocity, null);
  //   builder.addDoubleProperty("Angle (radians)", m_rotationEncoder::getPosition, null);
  // }

  /**
   * Set the constants for the rotation Spark Max's built in PID.
   */
  private void configureSparkMaxPID() {
    m_rotationPIDController.setP(rotationPID_kP);
    m_rotationPIDController.setI(rotationPID_kI);
    m_rotationPIDController.setD(rotationPID_kD);
    m_rotationPIDController.setIZone(rotationPID_IZone);
    m_rotationPIDController.setFF(rotationFF);
    m_rotationPIDController.setPositionPIDWrappingEnabled(true);
    m_rotationPIDController.setPositionPIDWrappingMinInput(-Math.PI);
    m_rotationPIDController.setPositionPIDWrappingMaxInput(Math.PI);
  }

  /**
   * Sends the module's drive PID constants to NetworkTables and updates them
   * whenever they are changed.
   * 
   * Only active while the function is called. Call this function periodically
   * to display and update the drive PID constants.
   * 
   * Calling this function while in a match is not recommended, as it will slow
   * down the robot code.
   */
  public void displayDrivePID() {
    final double kP = SmartDashboard.getNumber(m_name + " Drive PID kP", m_drivePIDController.getP());
    final double kI = SmartDashboard.getNumber(m_name + " Drive PID kI", m_drivePIDController.getI());
    final double kD = SmartDashboard.getNumber(m_name + " Drive PID kD", m_drivePIDController.getD());
    if (kP != m_drivePIDController.getP()) m_drivePIDController.setP(kP);
    if (kI != m_drivePIDController.getI()) m_drivePIDController.setI(kI);
    if (kD != m_drivePIDController.getD()) m_drivePIDController.setD(kD);
  }

  /**
   * Sends the module's rotation feedforward constants to NetworkTables and
   * updates them whenever they are changed.
   * 
   * Only active while the function is called. Call this function periodically
   * to display and update the rotation feedforward constants.
   * 
   * Calling this function while in a match is not recommended, as it will slow
   * down the robot code.
   */
  public void displayRotationFF() {
    final double gain = SmartDashboard.getNumber(m_name + " Rotation FF Gain", m_rotationPIDController.getFF());
    if (gain != m_rotationPIDController.getFF()) m_rotationPIDController.setFF(gain);
  }

  /**
   * Sends the module's rotation PID constants to NetworkTables and updates the
   * PID's constants whenever they are changed.
   * 
   * Only active while the function is called. Call this function periodically
   * to display and update the rotation PID constants.
   * 
   * Calling this function while in a match is not recommended, as it will slow
   * down the robot code.
   */
  public void displayRotationPID() {
    final double kP = SmartDashboard.getNumber(m_name + " Rotation PID kP", m_rotationPIDController.getP());
    final double kI = SmartDashboard.getNumber(m_name + " Rotation PID kI", m_rotationPIDController.getI());
    final double kD = SmartDashboard.getNumber(m_name + " Rotation PID kD", m_rotationPIDController.getD());
    final double IZone = SmartDashboard.getNumber(m_name + " Rotation PID IZone",  m_rotationPIDController.getIZone());
    if (kP != m_rotationPIDController.getP()) m_rotationPIDController.setP(kP);
    if (kI != m_rotationPIDController.getI()) m_rotationPIDController.setI(kI);
    if (kD != m_rotationPIDController.getD()) m_rotationPIDController.setD(kD);
    if (IZone != m_rotationPIDController.getIZone()) m_rotationPIDController.setIZone(IZone);
    /* It is not necessary to send the values to NetworkTables because if the
     * values change from within the code, they will be overwritten by the
     * value stored in NetworkTables. If the values change because of a change
     * in NetworkTables, then NetworkTables will already have the value.
     */
  }

  /**
   * Returns Position of the module
   * 
   * @return Position of the module
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
      m_driveEncoder.getPosition(), 
      new Rotation2d(m_rotationEncoder.getPosition())
    );
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
   * Reset the rotation and drive encoder POSITIONS. DOES NOT HOME THE MODULE!
   * This should be used to reset the encoder position after manually homing
   * the module. (Does not reset encoder velocity.)
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
    /* If the robot is not being instructed to move, do not move any motors. 
     * This prevents the swerve module from returning to its original position
     * when the robot is not moving, which is the default behaviour of
     * ChassisSpeeds and SwerveModuleState.
     */
    if (Math.abs(desiredState.speedMetersPerSecond) < MODULE_SPEED_DEADBAND) {
      stop();
      return;
    }
    
    /* Invert the rotation setpoint because the modules spin clockwise when
     * the rotation setpoint is positive (clockwise-positive) whereas
     * SwerveModuleState specifies counterclockwise-positive angles.
     */
    SwerveModuleState invertedState =
        new SwerveModuleState(desiredState.speedMetersPerSecond, 
            new Rotation2d(-desiredState.angle.getRadians()));
    
    // Optimize the desired (inverted) state to avoid spinning further than 90 degrees
    SwerveModuleState optimizedState =
        SwerveModuleState.optimize(invertedState, new Rotation2d(m_rotationEncoder.getPosition()));

    // Calculate the drive output from the drive PID controller.
    final double driveOutput =
        m_drivePIDController.calculate(m_driveEncoder.getVelocity(), optimizedState.speedMetersPerSecond);
    final double driveFeedforward = m_driveFeedforward.calculate(optimizedState.speedMetersPerSecond);

    final double driveVoltage = driveOutput + driveFeedforward;
    SmartDashboard.putNumber(m_name + " Drive Voltage", driveVoltage);
    
    SmartDashboard.putNumber(m_name + " Drive Setpoint (m/s)", optimizedState.speedMetersPerSecond);
    SmartDashboard.putNumber(m_name + " Angle Setpoint (rad)", optimizedState.angle.getRadians());
   
    m_driveMotor.setVoltage(driveVoltage);
    m_rotationPIDController.setReference(optimizedState.angle.getRadians(), ControlType.kPosition);
  }

  /**
   * Rotate the module to the specified angle.
   * This method must be called periodically in order to function.
   * 
   * @param desiredAngle The desired angle of the module, in radians.
   */
  public void rotateTo(double desiredAngle) {
    SmartDashboard.putNumber(m_name + " Angle Setpoint (rad)", desiredAngle);
    m_rotationPIDController.setReference(desiredAngle, ControlType.kPosition);
  }

  /**
   * Drive the module at the specified velocity.
   * This method must be called periodically in order to function.
   * 
   * @param desiredVelocity The desired velocity in m/s.
   */
  public void setVelocity(double desiredVelocity) {
    final double velocity = m_driveEncoder.getVelocity();
    final double feedback = m_drivePIDController.calculate(velocity, desiredVelocity);
    final double feedforward = m_driveFeedforward.calculate(desiredVelocity);
    final double output = feedback+feedforward;
    SmartDashboard.putNumber(m_name + " Drive Setpoint (m/s)", desiredVelocity);
    SmartDashboard.putNumber(m_name + " Drive Voltage", output);
    m_driveMotor.setVoltage(output);
  }

  /**
   * Stop all motors in this module.
   */
  public void stop() {
    m_driveMotor.setVoltage(0);
    m_rotationMotor.setVoltage(0);
  }

}