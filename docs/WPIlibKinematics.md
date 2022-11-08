# WPILib Classes for Swerve Drive

Documentation for these classes can be found at [docs.wpilib.org](https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/).

## ChassisSpeeds
`ChassisSpeeds` contains three components: `vx` (forward velocity), `vy` (sideways velocity), and `omega` (angular velocity in radians/second). A non-holonomic drivetrain always has a `vy` of 0.

`ChassisSpeeds` is constructed with 3 arguments: the forward velocity, sideways velocity, and angular velocity (`vx`, `vy`, and `omega`).

Radians can be converted into degrees by multiplying by 180 degrees / PI. Degrees can be converted into radians by multiplying by PI / 180 degrees.

The velocity must be specified in m/s in Java (C++ has the units library and can use any linear velocity unit).

`ChassisSpeeds` can be created using input from a controller:

```java
// XboxController.Axis class allows accessing different controller axis
int forwardVelocity = controller.Axis.kLeftY;
int sidewaysVelocity = controller.Axis.kLeftX;
int rotationVelocity = controller.Axis.kRightX;
// rotationVelocity can be multiplied to change rotational speed (e.g. 2 for doubling it or 0.5 for halving it)
ChassisSpeeds speeds = new ChassisSpeeds(forwardVelocity, sidewaysVelocity, rotationVelocity);
```

The `ChassisSpeeds.fromFieldRelativeSpeeds()` static method can be used for field-oriented drive. It accepts the same parameters as the normal constructor with the linear velocity relative to the field instead of the robot, and returns a `ChassisSpeeds` object:

```java
int forwardVelocity = controller.Axis.kLeftY;
int sidewaysVelocity = controller.Axis.kLeftX;
int rotationVelocity = controller.Axis.kRightX;
ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(forwardVelocity, sidewaysVelocity, rotationVelocity);
```

## SwerveModuleState
`SwerveModuleState` contains the speed and angle of a single swerve drive module.

Its constructor takes in the wheel's velocity (in m/s), and the angle of the wheel (in radians).

An angle of 0 means the modules are facing forward.

Module states can be converted to and from `ChassisSpeeds`. Construct a `SwerveDriveKinematics` to convert `ChassisSpeeds` -> `SwerveModuleState`, and use the `SwerveDriveKinematics.toChassisSpeeds()` (takes in each of the `SwerveModuleState` objects) static method to convert `SwerveModuleState` -> `ChassisSpeeds`.

## SwerveDriveKinematics
`SwerveDriveKinematics` is constructed by providing several `Translation2d` objects. One `Translation2d` object should be provided for each swerve module, and there must be at least 2 modules.
Provide a Translation2d (instructions on how to do it below) for each of the four swerve modules.

### Translation2d
We are using Java, so all units must be specified in metres.

`Translation2d` contains an `x` and a `y` coordinate (the location of the swerve module relative to the centre of the robot). Both `x` and `y` are doubles.

These are the directions that different values of `x` and `y` correspond to:
```
              Positive X (Front)
                      ^
                      |
Positive Y (Left) <-------> Negative Y (Right)
                      |
                      v
              Negative X (Rear)
```

Check out [the WPILib reference](https://first.wpi.edu/wpilib/allwpilib/docs/release/java/edu/wpi/first/math/geometry/Translation2d.html) for other methods that work with `Translation2d` (as well as `Rotation2d` and `Pose2d` objects) which may be useful.

**Back to SwerveDriveKinematics:**

To find the speed and velocity any swerve object must be set to, use the `toSwerveModuleState()` method. It takes in a ChassisSpeeds object and returns an array of `SwerveModuleState` objects in the same order as the `SwerveDriveKinematics` object was constructed:

```java
SwerveDriveKinematics kinematics = new SwerveDriveKinematics(new Translation2d(0.25, 0.35), new Translation2d(-0.25, 0.35), new Translation2d(0.25, -0.35), new Translation2d(-0.25, -0.35));
// our example robot is a 0.5m x 0.7m rectangle with swerve modules 0.25m x 0.35m from the centre
// assume order of FL, FR, RL, RR

ChassisSpeeds speeds = new ChassisSpeeds(1.0, 3.0, 1.5); // 1m/s forward, 3.0m/s to the left, 1.5 radians/s counterclockwise rotation

SwerveModuleState[] moduleStates = kinematics.toSwerveModuleState(speeds);

// give these to the drive motor and the pid controlling wheel angle
SwerveModuleState frontLeft = moduleStates[0];
SwerveModuleState frontRight = moduleStates[1];
SwerveModuleState rearLeft = moduleStates[2];
SwerveModuleState rearRight = moduleStates[3];
```

`toSwerveModuleStates()` takes in an optional second `Translation2d` parameter specifying a specific point of rotation. This allows more complex manouvers by rotating relative to some point on the robot (such as one corner instead of the centre).

`desaturateWheelSpeeds()` takes in an array of `SwerveModuleState` objects and a `double` representing the max speed in meters per second. It redistributes all speeds in a specific range if any exceed the max speed. For example, if the swerve module speeds are 0.5, 1, 1.5, and 2, with a max speed of 1, then each speed becomes 0.25, 0.5, 0.75, and 1. This limits the speed without distorting the ratio of speeds between each module.

### Optimizing Trajectories for Less Rotation
The static method `optimize()` of `SwerveModuleState` minimizes the change in heading of a swerve module, allowing the module to move a smaller distance (it will prevent the swerve module from moving more than 90 degrees at one time).

For example, a swerve drive module is at 45 degrees (turning towards the left-front). We need it go into reverse, so we tell it to move to 180 degrees, then set the drive motors to 1.0 speed. `optimize()` will see that the module only needs to move 90 degrees, because that is equivalent to 180 degrees if the drive motor spins in reverse. `optimize()` will set the drive speed to -1.0 and the desired angle to 90 degrees, which will propel the robot in the same direction, but spend less time moving the swerve module.

It takes the desired state of the module (a `SwerveModuleState`) and a `Rotation2d` (see below). It returns an optimized state you can use for the module's angle control loop.

```java
SwerveModuleState frontLeftOptimized = SwerveModuleState.optimize(frontLeft, new Rotation2d(m_turningEncoder.getDistance()));
// parameters: current SwerveModuleState, current position (in radians)
```

### Rotation2d
A `Rotation2d` object specifies a rotation, in radians. The constructor takes in a radian, but the object can optionally be constructed with the `Rotation2d.fromDegrees()` static method, which takes in an angle, in degrees and returns the Rotation2d object with the angle converted to radians.

Positive rotations are counterclockwise (left), and negative rotations are clockwise (right).

### Field-Oriented Drive
`ChassisSpeeds.fromFieldRelativeSpeeds()` can be used to create a ChassisSpeeds object from field relative speeds, which can be converted to an array of SwerveModuleStates:

```java
ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(2.0, 2.0, Math.PI / 2.0, Rotation2d.fromDegrees(45.0)); // moving 2.0m/s forward and to the left, rotating at 90 degrees per second (0.5 radians)

SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(speeds);
```

## Swerve Drive Odometry
Note: This uses only encoders and a gyro, meaning that the robot's position will eventually drift, especially as it is hit by other robots. It remains accurate in autonomous mode.

A `SwerveDriveOdometry` object is constructed by passing 3 arguments: a `SwerveDriveKinematics` object representing your swerve drive, a `Rotation2d` representing the angle of your robot (from the gyroscope), and an optional `Pose2d` argument representing the robot's starting position.

An angle of 0 degrees means the robot is facing the opponent drive station. A positive value means the robot is turning towards the left. This is the opposite behaviour of WPILib gyros, so placing a `-` on the gyroscope value is recommended.

The `update` method is used to update the robot odometry's position. It takes in a `Rotation2d` from the gyro of the robot (use the `getRotation2d` method), and each `SwerveModuleState` (in the same order as the `SwerveDriveKinematics` object was constructed). This method should be used in the `periodic()` function of a subsystem so it is updated frequently.

```java
Rotation2d gyroAngle = m_gyro.getRotation2d();
// getRotation2d() converts the gyro angle to ccw-positive and makes it continous (360 -> 361 instead of 360 -> 0)
// if you aren't using getRotation2d(), make sure to manually invert the gyro angle and use Rotation2d.fromDegrees().
// m_pose looks something like this: private final Pose2d m_pose;
m_pose = m_odometry.update(gyroAngle, m_frontLeft.getState(), m_frontRight.getState(), m_rearLeft.getState(), m_rearRight.getState());
// getState() is a user-defined function that should return (see example)
```

The `resetPose` method is used to reset the pose of the robot. It takes in a `Translation2d` (the new field-relative pose) and a `Rotation2d` (the gyro angle). You must call this method whenever you reset the gyro's position.

### Pose2d
A `Pose2d` object has 3 parts: a `x` value, a `y` value, and a `Rotation2d` value (radians).
```java
// robot is facing forward, 5.0m along length of field and 13.5m along short end of field (position is relative to bottom left corner of your alliance's driver stations)
Pose2d robotPosition = new Pose2d(5.0, 13.5, new Rotation2d());
// alternatively:
Pose2d robotPosition2 = new Pose2d(new Translation2d(5.0, 13.5), new Rotation2d());
```

## PIDController

Todo:
- Check validity of Xbox controller functions
- Program an example (maybe display current wheel angles and speeds, as well as field-relative robot position on Shuffleboard)
- Make Java programming lessons
- Zero yaw button
- Navx isConnected() indicator on SmartDashboard