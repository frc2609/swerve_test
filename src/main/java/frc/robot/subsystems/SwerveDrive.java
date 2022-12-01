// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

// static imports allow access to all constants in the class without using its name
import static frc.robot.Constants.Swerve.*;
import frc.robot.Constants.Swerve.CanID;
import frc.robot.Constants.Swerve.Position;
import frc.robot.Constants.Xbox;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Controls all swerve drive modules.
 */
public class SwerveDrive extends SubsystemBase {
  private final AHRS m_gyro;
  private final XboxController m_driverController;

  // x and y are relative to robot (x front/rear, y left/right)
  private final SlewRateLimiter m_xSpeedLimiter = new SlewRateLimiter(X_SPEED_DELAY);
  private final SlewRateLimiter m_ySpeedLimiter = new SlewRateLimiter(Y_SPEED_DELAY);
  private final SlewRateLimiter m_rotationLimiter = new SlewRateLimiter(ROTATION_DELAY);

  private final Translation2d m_frontLeftLocation = new Translation2d(Position.frontLeftX, Position.frontLeftY);
  private final Translation2d m_frontRightLocation = new Translation2d(Position.frontRightX, Position.frontRightY);
  private final Translation2d m_rearLeftLocation = new Translation2d(Position.rearLeftX, Position.rearLeftY);
  private final Translation2d m_rearRightLocation = new Translation2d(Position.rearRightX, Position.rearRightY);
  
  private final SwerveModule m_frontLeft = new SwerveModule(CanID.frontLeftDrive, CanID.frontLeftRotation);
  private final SwerveModule m_frontRight = new SwerveModule(CanID.frontRightDrive, CanID.frontRightRotation);
  private final SwerveModule m_rearLeft = new SwerveModule(CanID.rearLeftDrive, CanID.rearLeftRotation);
  private final SwerveModule m_rearRight = new SwerveModule(CanID.rearRightDrive, CanID.rearRightRotation);

  private final SwerveDriveKinematics m_kinematics =
      new SwerveDriveKinematics(
          m_frontLeftLocation, m_frontRightLocation, m_rearLeftLocation, m_rearRightLocation);
  
  private final SwerveDriveOdometry m_odometry;
  
  /** Creates a new SwerveDrive. */
  public SwerveDrive(AHRS gyro, XboxController driverController) {
    m_driverController = driverController;
    m_gyro = gyro;
    if (!m_gyro.isConnected()) {
      DriverStation.reportError(
        "Navx not initialized: Could not setup SwerveDriveOdometry", false);
    }
    m_odometry = new SwerveDriveOdometry(m_kinematics, m_gyro.getRotation2d());
    resetModuleEncoders();
  }

  // This method will be called once per scheduler run.
  @Override
  public void periodic() {
    updateOdometry();
  }

  /** 
   * Drive the robot using given inputs.
   *
   * @param xSpeed Speed of the robot in m/s relative to the x axis of the 
   * field (or robot). (Forward/Back Speed)
   * @param ySpeed Speed of the robot in m/s relative to the y axis of the
   * field (or robot). (Left/Right Speed)
   * @param rotationSpeed How many rotations the robot does per second, in
   * radians. (Positive values are counterclockwise rotations.)
   * @param isFieldRelative Whether the robot's movement is relative to the
   * field or the robot frame.
   */
  public void drive(double xSpeed, double ySpeed, double rotationSpeed, boolean isFieldRelative) {
    // Find states using field relative position or robot relative position.
    SwerveModuleState[] states = 
        m_kinematics.toSwerveModuleStates(
            isFieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotationSpeed, m_gyro.getRotation2d())
                : new ChassisSpeeds(xSpeed, ySpeed, rotationSpeed));
    // Prevent robot from going faster than it should.
    SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_LINEAR_SPEED);
    // Array index order must match the order that m_kinematics was initialized with.
    m_frontLeft.setDesiredState(states[0]);
    m_frontRight.setDesiredState(states[1]);
    m_rearLeft.setDesiredState(states[2]);
    m_rearRight.setDesiredState(states[3]);
  }

  /**
   * Drive the robot using joystick inputs from the driver's Xbox controller 
   * (controller specified in class constructor).
   * 
   * @param isFieldRelative Whether the robot's movement is relative to the
   * field or the robot frame.
   */
  public void manualDrive(boolean isFieldRelative) {
    /* getLeftY() is used for xSpeed because xSpeed moves robot forward/back.
     * (The same applies for getLeftX()). This occurs because of the way robot
     * coordinates are implemented in WPILib.
     * (See https://docs.wpilib.org/en/stable/docs/software/advanced-controls/geometry/coordinate-systems.html.)
     */
    /* Speeds are inverted because Xbox controllers return negative values when
     * joystick is pushed forward or to the left.
     */
    final double xSpeed =
        -m_xSpeedLimiter.calculate(MathUtil.applyDeadband(
            m_driverController.getLeftY(), Xbox.JOYSTICK_DEADBAND))
                * MAX_LINEAR_SPEED; // m/s
                // scale value from 0-1 to 0-MAX_LINEAR_SPEED

    final double ySpeed =
        -m_ySpeedLimiter.calculate(MathUtil.applyDeadband(
            m_driverController.getLeftX(), Xbox.JOYSTICK_DEADBAND))
                * MAX_LINEAR_SPEED;

    final double rotationSpeed =
        -m_rotationLimiter.calculate(MathUtil.applyDeadband(
            m_driverController.getRightX(), Xbox.JOYSTICK_DEADBAND))
                * MAX_ANGULAR_VELOCITY; // radians / second

    drive(xSpeed, ySpeed, rotationSpeed, isFieldRelative);
  }

  /**
   * Reset the encoders of each module. DOES NOT HOME THE MODULE!
   * This should be used to reset the encoder position after manually homing
   * every module.
   * 
   * This is automatically called when the robot is powered up, so it is
   * unnecessary to call this if the modules are already homed when the robot
   * is turned on.
   */
  public void resetModuleEncoders() {
    m_frontLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearLeft.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /** Updates the field relative position of the robot. */
  public void updateOdometry() {
    m_odometry.update(
      m_gyro.getRotation2d(),
      m_frontLeft.getState(),
      m_frontRight.getState(),
      m_rearLeft.getState(),
      m_rearRight.getState());
  }
}