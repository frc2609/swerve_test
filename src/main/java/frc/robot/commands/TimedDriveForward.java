// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.subsystems.SwerveDrive;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TimedDriveForward extends ParallelDeadlineGroup {
  /** Creates a new TimedDriveForward. */
  public TimedDriveForward(double timerDelay, SwerveDrive swerveDrive) {
    // Add the deadline command in the super() call. Add other commands using
    // addCommands().
    super(new TimerDelay(timerDelay));
    addCommands(new DriveForwardWhileSpinning(swerveDrive, 2, Math.PI, 2));
  }
}
