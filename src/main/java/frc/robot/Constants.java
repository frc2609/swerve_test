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
//    /** [description here] (Named Dashboard to avoid conflict with Shuffleboard class.) */
//    public final class Dashboard {
//        public static final String gyroLayout = "NavX";
//        public static final String drivetrainTab = "Drivetrain";
//    }
    /** Swerve drive related constants. */
    public final class Swerve {
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
        // TODO: find values
        /** Swerve drive PID and feedforward gains. 
         * setVoltage() is used to set motor power, as it ensures the motor
         * always outputs the same force when the battery voltage sags. 
         * Since setVoltage() is being used, these gains are tuned to produce
         * a *voltage* value, not a speed value, so `set()` should not be used
         * with any controller using these gains.
         */
        public final class Gains {
            // TODO: tune
            public static final double drivePID_kP = 1.1867;
            public static final double drivePID_kI = 0;
            public static final double drivePID_kD = 0;

            public static final double rotationPID_kP = 0.4;
            public static final double rotationPID_kI = 0.0001;
            public static final double rotationPID_kD = 1.0;
            public static final double rotationPID_IZone = 0.001;

            // TODO: tune
            public static final double driveFF_kS = 0;
            public static final double driveFF_kV = 3.333;

            // SparkMaxPIDController only has 1 feedforward constant.
            public static final double rotationFF = 0.6;
        }
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
            public static final double TRACK_SIZE_X = 0.6096; // 24 inches (in metres)
            /** Left-right (Y) distance between two swerve modules measured from centre of wheel in metres. */
            public static final double TRACK_SIZE_Y = 0.6096; // 24 inches (in metres)
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
        /** Autonomous acceleration and speed limits. */
        // public final class AutonomousLimits {
        //     /** The maximum linear acceleration the robot should achieve in m/s^2. */
        //     public static final double MAX_LINEAR_ACCELERATION = 0; // TODO: adjust value
        //     /** The maximum speed the drivetrain should go in autonomous in m/s. */
        //     public static final double MAX_LINEAR_SPEED = 3.8;
        // }
        /** Teleop acceleration and speed limits */
        public final class TeleopLimits {
            /** The maximum speed the robot should spin in teleop in radians/s. */
            public static final double MAX_ANGULAR_VELOCITY = 4 * Math.PI; // TODO: adjust value
            /** The maximum speed the drivetrain should go in teleop in m/s. */
            public static final double MAX_LINEAR_SPEED = 3.8;
        }
        // Miscellaneous:
        public static final double DEBUG_DRIVE_ANGLE_SENSITIVITY = 0.25;
        /** The maximum angular acceleration the robot can achieve in radians/s^2. */
        // public static final double MAX_POSSIBLE_ANGULAR_ACCELERATION = 2 * Math.PI; // unused
        /** The maximum speed the robot can spin in radians/s. */
        // public static final double MAX_POSSIBLE_ANGULAR_VELOCITY = 4 * Math.PI; // unused
        /** The maximum linear speed a swerve module can achieve in m/s. */
        public static final double MAX_POSSIBLE_LINEAR_SPEED = 3.8;
        /** Any speeds below this value will not cause the module to move. */
        public static final double MODULE_SPEED_DEADBAND = 0.001; // m/s
        /** 56.6409 rotations of motor = 1.0 rotation of module 
         * <p>UltraPlanetary gearbox ratios differ from the ratio printed on
         * the gearbox's side. The motor is connected to a 5:1 gearbox and a
         * 4:1 gearbox, whose actual ratios are 5.23:1 and 3.61:1 respectively.
         * The module spins once for every 3 rotations of the UltraPlanetary's
         * output, which gives a gear ratio of 5.23 x 3.61 x 3 = 56.6409.
         */
        public static final double ROTATION_GEAR_RATIO = 1.0 / 56.6409; // .0 to avoid integer division
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
         * RPM * motor : wheel gear ratio = wheel RPM
         * Wheel RPM / 60 seconds/minute = wheel RPS
         * Wheel RPS * wheel circumference in metres = m/s.
         * Note: Actual equation is equivalent to procedure above, but simplified.
         */
        public static final double DRIVE_VELOCITY_CONVERSION = (WHEEL_GEAR_RATIO * WHEEL_CIRCUMFERENCE) / 60;
        /** Converts to module position in radians.
         * Explanation:
         * Spark Max outputs total rotations.
         * Total rotations * rotation gear ratio = module rotations (position).
         * Module rotations * (2 * PI) radians/rotation = rotations in radians (position).
         */
        public static final double ROTATION_POSITION_CONVERSION = 2 * Math.PI * ROTATION_GEAR_RATIO;
        /** Converts to metres per second.
         * Explanation:
         * Spark Max outputs RPM (rotations per minute).
         * RPM * motor : rotation gear ratio = module RPM
         * Module RPM / 60 seconds/minute = module RPS
         * Module RPS * 2 * Pi radians/rotation = radians/second.
         * Note: Actual equation is equivalent to procedure above, but simplified.
         */
        public static final double ROTATION_VELOCITY_CONVERSION = (2 * Math.PI * ROTATION_GEAR_RATIO) / 60;
    }
    /** 
     * Xbox controller related constants. 
     * Do not put button or axis numbers in here, instead use the functions it
     * provides, such as getLeftY() or getXButton().
     */
    public final class Xbox {
        public static final int DRIVER_CONTROLLER_PORT = 0;
        public static final double JOYSTICK_DEADBAND = 0.075;
    }
}
