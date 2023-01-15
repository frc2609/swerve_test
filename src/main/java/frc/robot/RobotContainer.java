// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/*
* I tried to program a Shuffleboard layout within the program, however some
* elements don't show up (seemingly randomly) so I've disabled it.
* (This is why there are so many commented out lines of code.)
*/

package frc.robot;

import com.kauailabs.navx.frc.AHRS;

//import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
// import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
// import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
// import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
// import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
// import frc.robot.Constants.Dashboard;
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

  public final SwerveDrive m_swerveDrive;
  
  // private final JoystickButton m_fieldOrientedToggleButton = 
  //     new JoystickButton(
  //         m_driverController, XboxController.Button.kBack.value);
  // private final JoystickButton m_resetEncoderButton =
  //     new JoystickButton(
  //         m_driverController, XboxController.Button.kStart.value);
  private final JoystickButton m_zeroYawButton =
      new JoystickButton(
          m_driverController, XboxController.Button.kY.value);
/*
  // Hardcoded Shuffleboard layout did not work.
  private final ShuffleboardTab drivetrainTab = Shuffleboard.getTab(Dashboard.drivetrainTab);
  // private final ShuffleboardLayout gyroLayout =
  //     drivetrainTab
  //     .getLayout(Dashboard.gyroLayout, BuiltInLayouts.kGrid)
  //     .withPosition(0, 0)
  //     .withSize(2, 3)
  //     .withProperties(Map.of("Title", "NavX", "number of columns", 2, "number of rows", 3));
  private final NetworkTableEntry gyroAngle = 
      //gyroLayout
      drivetrainTab
      .add("Robot Heading", 0)
      .withWidget(BuiltInWidgets.kGyro)
      .withPosition(0, 0)
      .withSize(2, 2)
      .getEntry();
      // these two do not appear now because shuffleboard reasons
  private final NetworkTableEntry isConnected =
      //gyroLayout
      drivetrainTab
      .add("NavX Connected", false)
      .withWidget(BuiltInWidgets.kBooleanBox)
      .withPosition(0, 2) // correct
      .withSize(1, 1) // correct
      .getEntry();
  private final NetworkTableEntry zeroYaw =
      //gyroLayout
      drivetrainTab
      .add("Zero Yaw", false)
      .withWidget(BuiltInWidgets.kToggleButton)
      .withPosition(1, 2) // correct
      .withSize(1, 1) // correct
      .getEntry();
*/

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    try {
      m_navx = new AHRS(SerialPort.Port.kMXP);
    } catch (RuntimeException e) {
      DriverStation.reportError("Navx initialization failed", false);
    }
    m_swerveDrive = new SwerveDrive(m_navx, m_driverController);
    configureButtonBindings();
    SmartDashboard.putBoolean("Zero Yaw", false); // display the buttonx
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // replaced with SmartDashboard buttons
    // m_fieldOrientedToggleButton.onTrue(new InstantCommand(
    //   () -> m_isFieldRelative = !m_isFieldRelative));
    // m_resetEncoderButton.onTrue(new InstantCommand(
    //   m_swerveDrive::resetModuleEncoders, m_swerveDrive));
    m_zeroYawButton.onTrue(new InstantCommand(m_navx::zeroYaw));
    // this one left in for easy access to resetYaw
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
   * Update NetworkTables values set by RobotContainer.
   */
  public void updateNetworkTables() {
    SmartDashboard.putNumber("Gyro Angle", m_navx.getAngle());
    SmartDashboard.putBoolean("Navx Connected", m_navx.isConnected());
    if (SmartDashboard.getBoolean("Zero Yaw", false)) {
      m_navx.zeroYaw();
      SmartDashboard.putBoolean("Zero Yaw", false); // reset the button
    }
  }
  // public void updateNetworkTables() {
  //   gyroAngle.setDouble(m_navx.getAngle());
  //   isConnected.setBoolean(m_navx.isConnected());
  //   if (zeroYaw.getBoolean(false)) {
  //     m_navx.zeroYaw();
  //     zeroYaw.setBoolean(false); // reset the button
  //   }
  // }
}
