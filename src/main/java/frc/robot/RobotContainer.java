// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.SwerveDrive;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private static AHRS m_navx;
  public final XboxController m_driverController = new XboxController(
    Constants.Xbox.DRIVER_CONTROLLER_PORT);

  private boolean m_isFieldRelative = true; 
  // Note: This should be removed and hardcoded to be true on the competition robot.
  // Should it? SmartDashboard toggle is probably better (default true) so that it can be turned off for testing.

  public final SwerveDrive m_swerveDrive;

  private final JoystickButton m_fieldOrientedToggleButton = 
      new JoystickButton(
          m_driverController, XboxController.Button.kBack.value);
  private final JoystickButton m_resetEncoderButton =
      new JoystickButton(
          m_driverController, XboxController.Button.kStart.value);
  private final JoystickButton m_zeroYawButton =
      new JoystickButton(
          m_driverController, XboxController.Button.kY.value);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    try {
      m_navx = new AHRS(SerialPort.Port.kMXP);
    } catch (RuntimeException e) {
      DriverStation.reportError("Navx initialization failed", false);
    }
    m_swerveDrive = new SwerveDrive(m_navx, m_driverController);
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    m_fieldOrientedToggleButton.whenPressed(new InstantCommand(
      () -> m_isFieldRelative = !m_isFieldRelative));
    m_resetEncoderButton.whenPressed(new InstantCommand(
      m_swerveDrive::resetModuleEncoders, m_swerveDrive));
    m_zeroYawButton.whenPressed(new InstantCommand(m_navx::zeroYaw));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return null;
  }

  /**
   * Returns whether field-relative/field-oriented mode is enabled.
   */
  public boolean isFieldRelative() {
    return m_isFieldRelative;
  }
}
