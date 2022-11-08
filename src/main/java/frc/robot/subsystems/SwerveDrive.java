// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class SwerveDrive extends SubsystemBase {
  // TODO: measure distance between each swerve module
  private final Translation2d m_frontLeftLocation = new Translation2d(0.381, 0.381);
  private final Translation2d m_frontRightLocation = new Translation2d(0.381, -0.381);
  private final Translation2d m_rearLeftLocation = new Translation2d(-0.381, 0.381);
  private final Translation2d m_rearRightLocation = new Translation2d(-0.381, -0.381);
  
  // TODO: find DIO port numbers
  private final SwerveModule m_frontLeft = new SwerveModule(Constants.SwerveID.frontLeftDrive, Constants.SwerveID.frontLeftRotation, 0);
  private final SwerveModule m_frontRight = new SwerveModule(Constants.SwerveID.frontRightDrive, Constants.SwerveID.frontRightRotation, 0);
  private final SwerveModule m_rearLeft = new SwerveModule(Constants.SwerveID.rearLeftDrive, Constants.SwerveID.rearLeftRotation, 0);
  private final SwerveModule m_rearRight = new SwerveModule(Constants.SwerveID.rearRightDrive, Constants.SwerveID.rearRightRotation, 0);

  private final SwerveDriveKinematics m_kinematics =
      new SwerveDriveKinematics(
          m_frontLeftLocation, m_frontRightLocation, m_rearLeftLocation, m_rearRightLocation);
  
  private final SwerveDriveOdometry m_odometry =
      new SwerveDriveOdometry(m_kinematics, RobotContainer.gyro().getRotation2d());
  
  /** Creates a new SwerveDrive. */
  public SwerveDrive() {}

  /** 
   * Drive the robot using joystick input.
   *
   * @param xSpeed Speed of the robot in m/s relative to the x axis of the field (or robot).
   * @param ySpeed Speed of the robot in m/s relative to the y axis of the field (or robot).
   * @param rotationSpeed How many rotations the robot does per second, in radians.
   * @param isFieldOriented Whether the robot's movement is relative to the field or the robot frame.
   */
  public void drive(double xSpeed, double ySpeed, double rotationSpeed, boolean isFieldRelative) {
    // Find states using field relative position or robot relative position.
    SwerveModuleState[] states = 
        m_kinematics.toSwerveModuleStates(
            isFieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotationSpeed, RobotContainer.gyro().getRotation2d())
                : new ChassisSpeeds(xSpeed, ySpeed, rotationSpeed));
    // Prevent robot from going faster than it should.
    SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.SwerveDrive.MAX_LINEAR_SPEED);
    // Array index order must match the order that m_kinematics was initialized with.
    m_frontLeft.setDesiredState(states[0]);
    m_frontRight.setDesiredState(states[1]);
    m_rearLeft.setDesiredState(states[2]);
    m_rearRight.setDesiredState(states[3]);
  }

  /** Updates the field relative position of the robot. */
  public void updateOdometry() {
    m_odometry.update(
      RobotContainer.gyro().getRotation2d(),
      m_frontLeft.getState(),
      m_frontRight.getState(),
      m_rearLeft.getState(),
      m_rearRight.getState());
  }
}
