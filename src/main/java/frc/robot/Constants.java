// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final double TRACK_WIDTH_METRES = 0.6;
    public static final double TRACK_LENGTH_METRES = 0.9;
    public final class SwerveDrive {
        public static final double MAX_ANGULAR_SPEED = Math.PI; // radians
        public static final double MAX_LINEAR_SPEED = 3.0; // m/s
    }
    public final class SwerveID {
        public static final int frontLeftDrive = 1;
        public static final int frontLeftRotation = 2;
        public static final int frontRightDrive = 3;
        public static final int frontRightRotation = 4;
        public static final int rearLeftDrive = 5;
        public static final int rearLeftRotation = 6;
        public static final int rearRightDrive = 7;
        public static final int rearRightRotation = 8;
    }
    public final class Xbox {
        public static final int DRIVER_PORT = 0;
        public static final int LEFT_STICK_X_AXIS = 0;
        public static final int LEFT_STICK_Y_AXIS = 1;
        public static final int RIGHT_STICK_X_AXIS = 4;
    }
}
