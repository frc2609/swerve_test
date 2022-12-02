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
    /** Swerve drive related constants. */
    public final class Swerve {
        // TODO: assign values to Spark Maxes
        /** CAN IDs of swerve module motor controllers. */
        public final class CanID {
            public static final int frontLeftDrive = 1;
            public static final int frontLeftRotation = 2;
            public static final int frontRightDrive = 3;
            public static final int frontRightRotation = 4;
            public static final int rearLeftDrive = 5;
            public static final int rearLeftRotation = 6;
            public static final int rearRightDrive = 7;
            public static final int rearRightRotation = 8;
        }
        // TODO: find values -> sysID will save lots of time
        /** Swerve drive PID and feedforward gains. */
        public final class Gains {
            public static final double drivePID_kP = 1;
            public static final double drivePID_kI = 0;
            public static final double drivePID_kD = 0;

            public static final double rotationPID_kP = 1;
            public static final double rotationPID_kI = 0;
            public static final double rotationPID_kD = 0;

            public static final double driveFF_kS = 1;
            public static final double driveFF_kV = 1;

            public static final double rotationFF_kS = 1;
            public static final double rotationFF_kV = 1;
        }
        // TODO: find values, update
        /** Constants related to swerve module positions. */
        public final class Position {
            /* Direction Polarity:
                      +X (Front)
                 FL       ^       FR
                          |
            +Y (Left) <-------> -Y (Right)
                          |
                 RL       v       RR
                      -X (Rear)
             * Direction  | X | Y |
             * Front Left   +   +
             * Front Right  +   -
             * Rear Left    -   +
             * Rear Right   -   -
             */
            /** Front-rear (X) distance between two swerve modules measured from centre of wheel in metres. */
            public static final double TRACK_SIZE_X = 0.9; // TODO: find
            /** Left-right (Y) distance between two swerve modules measured from centre of wheel in metres. */
            public static final double TRACK_SIZE_Y = 0.6; // TODO: find
            /** Front-rear (X) distance from centre of wheel to centre of robot in metres. */
            public static final double X_FROM_CENTRE = TRACK_SIZE_X / 2;
            /** Left-right (Y) distance from centre of wheel to centre of robot in metres. */
            public static final double Y_FROM_CENTRE = TRACK_SIZE_Y / 2;
            public static final double frontLeftX   = X_FROM_CENTRE;
            public static final double frontLeftY   = Y_FROM_CENTRE;
            public static final double frontRightX  = X_FROM_CENTRE;
            public static final double frontRightY  = -Y_FROM_CENTRE;
            public static final double rearLeftX    = -X_FROM_CENTRE;
            public static final double rearLeftY    = Y_FROM_CENTRE;
            public static final double rearRightX   = -X_FROM_CENTRE;
            public static final double rearRightY   = -Y_FROM_CENTRE;
        }
        // Miscellaneous:
        public static final double MAX_ANGULAR_ACCELERATION = 2 * Math.PI; // radians per second squared, TODO: adjust value
        public static final double MAX_ANGULAR_VELOCITY = Math.PI; // radians, TODO: adjust value
        public static final double MAX_LINEAR_SPEED = 3.0; // m/s, TODO: adjust value
        /** 60.0 rotations of motor = 1.0 rotation of module */
        public static final double ROTATION_GEAR_RATIO = 1.0 / 60.0; // .0 to avoid integer division
        public static final double WHEEL_RADIUS = 0.0508; // metres
        public static final double WHEEL_CIRCUMFERENCE = 2 * Math.PI * WHEEL_RADIUS; // metres
        /** 8.0 rotations of motor = 1.0 rotation of wheel */
        public static final double WHEEL_GEAR_RATIO = 1.0 / 8.0; // .0 to avoid integer division
        // Limiters:
        public static final double X_SPEED_DELAY = 3; // 1/x seconds from 0 -> 1
        public static final double Y_SPEED_DELAY = 3; // 1/x seconds from 0 -> 1
        public static final double ROTATION_DELAY = 3; // 1/x seconds from 0 -> 1
        /* Conversions:
         * Spark Max encoders return rotations for getPosition() and rotations/minute for getVelocity().
         * Their return value is automatically multiplied by their conversion factor.
         */
        /** Converts to distance travelled in metres.
         * Explanation:
         * Spark Max outputs total rotations.
         * Total rotations * circumference of wheel * wheel gear ratio = distance travelled in metres.
         */
        public static final double DRIVE_POSITION_CONVERSION = WHEEL_GEAR_RATIO * WHEEL_CIRCUMFERENCE;
        /** Converts to metres per second.
         * Explanation:
         * Spark Max outputs RPM (rotations per minute).
         * RPM * 60 seconds/minute = RPS (rotations per second).
         * RPS * motor : wheel gear ratio = wheel RPS
         * Wheel RPS * wheel circumference in metres = m/s.
         */
        public static final double DRIVE_VELOCITY_CONVERSION = 60 * WHEEL_GEAR_RATIO * WHEEL_CIRCUMFERENCE;
        /** Converts to module position in radians.
         * Explanation:
         * Spark Max outputs total rotations.
         * Total rotations * rotation gear ratio = module rotations (position).
         * Module rotations * (2 * PI) radians/rotation = rotations in radians (position).
         */
        public static final double ROTATION_POSITION_CONVERSION = 2 * Math.PI * ROTATION_GEAR_RATIO;
    }
    /** 
     * Xbox controller related constants. 
     * Do not put button or axis numbers in here, instead use the functions it
     * provides, such as getLeftY() or getXButton().
    */
    public final class Xbox {
        public static final int DRIVER_CONTROLLER_PORT = 0;
        public static final double JOYSTICK_DEADBAND = 0.05;
    }
}
