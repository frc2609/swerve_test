// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrive;

public class DriveForwardWhileSpinning extends CommandBase {
  private final SwerveDrive m_swerveDrive;
  private final double m_linearSpeed;
  private final double m_rotationSpeedRad;

  /** Creates a new DriveForwardWhileSpinning. */
  public DriveForwardWhileSpinning(
      SwerveDrive swerveDrive,
      double linearSpeed,
      double rotationSpeedRad,
      double timerDelay
  ) {
    m_swerveDrive = swerveDrive;
    m_rotationSpeedRad = rotationSpeedRad;
    m_linearSpeed = linearSpeed;
    addRequirements(swerveDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_swerveDrive.getPose();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_swerveDrive.drive(m_linearSpeed, 0, m_rotationSpeedRad, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_swerveDrive.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}