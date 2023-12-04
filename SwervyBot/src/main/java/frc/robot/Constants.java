// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;
import java.util.stream.Collectors;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.lib.util.BetterSwerveKinematics;
import frc.lib.util.SwerveModuleConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final Translation3d FIELD_CENTER = new Translation3d(
        Units.feetToMeters(27),
        Units.feetToMeters(13.5),
        0);
    
    public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
    public static final double MANIPULATOR_MASS = 10 * 0.453592; // 10lbs * kg per pound
    public static final double CHASSIS_MASS = ROBOT_MASS - MANIPULATOR_MASS;
    public static final Translation3d CHASSIS_CG = new Translation3d(
        0,
        0,
        Units.inchesToMeters(8));
    public static final double ARM_Y_POS = 0; // centered on robot
    public static final double GRAVITY = 9.81; // m/s/s
    public static final double SPARK_TOTAL_LAG_TIME = 0.13; //s, 20ms + 110ms sprk max velocity lag
    public static final double RIO_LOOP_TIME = 0.02;

    public static final double dummyArmX = Units.inchesToMeters(42);
    public static final double dummyArmHieght = Units.inchesToMeters(27);

    public static final double FIELD_WIDTH = Units.inchesToMeters((12 * 26) + 3.5);

    public static final class Auton {

        public static final double X_KP = 4;
        public static final double X_KI = 0;
        public static final double X_KD = 0;

        public static final double Y_KP = 4;
        public static final double Y_KI = 0;
        public static final double Y_KD = 0;

        public static final double ANG_KP = 8;
        public static final double ANG_KI = 0;
        public static final double ANG_KD = 0;

        public static final double MAX_SPEED = Units.feetToMeters(14.5);
        public static final double MAX_ACCELERATION = 8;

        private static final Map<String, Pose2d> BLUE_MAP = Map.ofEntries(
            Map.entry("Node1High", new Pose2d(new Translation2d(1.582 + 1, Units.inchesToMeters(19.875)), Rotation2d.fromDegrees(180))),
            Map.entry("Node2High", new Pose2d(new Translation2d(1.582 + 1, Units.inchesToMeters(41.875)), Rotation2d.fromDegrees(180))),
            Map.entry("Node3High", new Pose2d(new Translation2d(1.582 + 1, Units.inchesToMeters(63.875)), Rotation2d.fromDegrees(180))),
            Map.entry("Node4High", new Pose2d(new Translation2d(1.582 + 1, Units.inchesToMeters(85.875)), Rotation2d.fromDegrees(180))),
            Map.entry("Node5High", new Pose2d(new Translation2d(1.582 + 1, Units.inchesToMeters(107.875)), Rotation2d.fromDegrees(180))),
            Map.entry("Node6High", new Pose2d(new Translation2d(1.582 + 1, Units.inchesToMeters(129.875)), Rotation2d.fromDegrees(180))),
            Map.entry("Node7High", new Pose2d(new Translation2d(1.582 + 1, Units.inchesToMeters(151.875)), Rotation2d.fromDegrees(180))),
            Map.entry("Node8High", new Pose2d(new Translation2d(1.582 + 1, Units.inchesToMeters(173.875)), Rotation2d.fromDegrees(180))),
            Map.entry("Node9High", new Pose2d(new Translation2d(1.582 + 1, Units.inchesToMeters(195.875)), Rotation2d.fromDegrees(180))),
            Map.entry("Node1Mid", new Pose2d(new Translation2d(), Rotation2d.fromDegrees(180))),
            Map.entry("Node2Mid", new Pose2d(new Translation2d(), Rotation2d.fromDegrees(180))),
            Map.entry("Node3Mid", new Pose2d(new Translation2d(), Rotation2d.fromDegrees(180))),
            Map.entry("Node4Mid", new Pose2d(new Translation2d(), Rotation2d.fromDegrees(180))),
            Map.entry("Node5Mid", new Pose2d(new Translation2d(), Rotation2d.fromDegrees(180))),
            Map.entry("Node6Mid", new Pose2d(new Translation2d(), Rotation2d.fromDegrees(180))),
            Map.entry("Node7Mid", new Pose2d(new Translation2d(), Rotation2d.fromDegrees(180))),
            Map.entry("Node8Mid", new Pose2d(new Translation2d(), Rotation2d.fromDegrees(180))),
            Map.entry("Node9Mid", new Pose2d(new Translation2d(), Rotation2d.fromDegrees(180))),
            Map.entry("Node1Low", new Pose2d(new Translation2d(), Rotation2d.fromDegrees(180))),
            Map.entry("Node2Low", new Pose2d(new Translation2d(), Rotation2d.fromDegrees(180))),
            Map.entry("Node3Low", new Pose2d(new Translation2d(), Rotation2d.fromDegrees(180))),
            Map.entry("Node4Low", new Pose2d(new Translation2d(), Rotation2d.fromDegrees(180))),
            Map.entry("Node5Low", new Pose2d(new Translation2d(), Rotation2d.fromDegrees(180))),
            Map.entry("Node6Low", new Pose2d(new Translation2d(), Rotation2d.fromDegrees(180))),
            Map.entry("Node7Low", new Pose2d(new Translation2d(), Rotation2d.fromDegrees(180))),
            Map.entry("Node8Low", new Pose2d(new Translation2d(), Rotation2d.fromDegrees(180))),
            Map.entry("Node9Low", new Pose2d(new Translation2d(), Rotation2d.fromDegrees(180)))
        );
        private static final Map<String, Pose2d> RED_MAP =
            BLUE_MAP.entrySet().stream().collect(Collectors.toMap(
                entry -> entry.getKey(),
                entry -> new Pose2d(
                    new Translation2d(
                        entry.getValue().getX(),
                        FIELD_WIDTH - entry.getValue().getY()),
                    entry.getValue().getRotation())));
        
        public static final Map<Alliance, Map<String, Pose2d>> POSE_MAP = Map.of(
            Alliance.Blue, BLUE_MAP,
            Alliance.Red, RED_MAP
        );

        // meters and radians
        public static final double X_TOLERANCE = 0.02;
        public static final double Y_TOLERANCE = 0.02;
        public static final double ANG_TOLERANCE = 0.5;
    }
    public static final class Drivebase {
        // Hold time on motor brakes when disabled
        public static final double WHEEL_LOCK_TIME = 10; // seconds

        public static final double HEADING_TOLERANCE = Math.toRadians(2);
        
        // Robot heading control gains
        public static final double HEADING_KP = 0.4;
        public static final double HEADING_KI = 0;
        public static final double HEADING_KD = 0;

        // Motor and encoder inversions
        public static final boolean CANCODER_INVERT = false;
        public static final boolean DRIVE_MOTOR_INVERT = false;
        public static final boolean ANGLE_MOTOR_INVERT = false;
        public static final boolean INVERT_GYRO = false;

        // Module locations, in meters, as distances to the center of the robot.
        // Positive x is torwards the robot front, and +y is torwards robot left.
        public static final double FRONT_LEFT_X = Units.inchesToMeters(11);
        public static final double FRONT_LEFT_Y = Units.inchesToMeters(11);
        public static final double FRONT_RIGHT_X = Units.inchesToMeters(11);
        public static final double FRONT_RIGHT_Y = Units.inchesToMeters(-11);
        public static final double BACK_LEFT_X = Units.inchesToMeters(-11);
        public static final double BACK_LEFT_Y = Units.inchesToMeters(11);
        public static final double BACK_RIGHT_X = Units.inchesToMeters(-11);
        public static final double BACK_RIGHT_Y = Units.inchesToMeters(-11);

        public static final Translation2d[] MODULE_LOCATIONS = {
            new Translation2d(Drivebase.FRONT_LEFT_X, Drivebase.FRONT_LEFT_Y),
            new Translation2d(Drivebase.FRONT_RIGHT_X, Drivebase.FRONT_RIGHT_Y),
            new Translation2d(Drivebase.BACK_LEFT_X, Drivebase.BACK_LEFT_Y),
            new Translation2d(Drivebase.BACK_RIGHT_X, Drivebase.BACK_RIGHT_Y)
        };

        // Drivetrain limitations
        public static final double MAX_SPEED = Units.feetToMeters(14.5); // meters per second
        public static final double MAX_ANGULAR_VELOCITY = MAX_SPEED / Math.hypot(FRONT_LEFT_X, FRONT_LEFT_Y); // rad/s
        // Theoretical max acceleration should be as follows:
        // (NEO stall torque * module gearing * number of modules) / (wheel radius * robot mass) = m/s/s
        // (2.6 * 6.75 * 4) / (Units.inchesToMeters(2) * ROBOT_MASS)
        // However, the drive is traction-limited, so the max accelration is actually (wheel coefficient of friction * gravity)
        public static final double MAX_ACCELERATION = 1.19 * 9.81; // COF (blue nitrile on carpet) as reported by Studica
        // max speed (RPM) / gear ratio, convert to deg/min, divide by 60 for deg/s
        public static final double MAX_MODULE_ANGULAR_SPEED = 360 * (5676 / 12.8) / 60; // deg/s
        public static final double MAX_ANGULAR_ACCELERATION = MAX_ACCELERATION / Math.hypot(FRONT_LEFT_X, FRONT_LEFT_Y);

        // Swerve base kinematics object
        public static final BetterSwerveKinematics KINEMATICS = new BetterSwerveKinematics(MODULE_LOCATIONS);

        // Module PIDF gains
        public static final double MODULE_KP = 0.01;
        public static final double MODULE_KI = 0;
        public static final double MODULE_KD = 0;
        public static final double MODULE_IZ = 0;
        public static final double MODULE_KF = 0;
        // Volt * seconds / degree.  Equal to (maxVolts) / (maxSpeed)
        public static final double MODULE_KV = 12 / MAX_MODULE_ANGULAR_SPEED;

        public static final double VELOCITY_KP = 0.0020645; // kp from SysId, eventually
        public static final double VELOCITY_KI = 0; // Leave all of these zero to disable them
        public static final double VELOCITY_KD = 0;
        public static final double VELOCITY_IZ = 0;
        public static final double VELOCITY_KF = 0;

        // Drive feedforward gains
        public static final double KS = 0;
        public static final double KV = 12 / MAX_SPEED; // Volt-seconds per meter (max voltage divided by max speed)
        public static final double KA = 12 / MAX_ACCELERATION; // Volt-seconds^2 per meter (max voltage divided by max accel)

        // Encoder conversion values.  Drive converts motor rotations to linear wheel distance
        // and steering converts motor rotations to module azimuth
        public static final double METERS_PER_MOTOR_ROTATION = (Math.PI * Units.inchesToMeters(4)) / 6.75;
            // Calculation: 4in diameter wheels * pi [circumfrence] / gear ratio
        public static final double DEGREES_PER_STEERING_ROTATION = 360 / 12.8;
            // degrees per rotation / gear ratio between module and motor
        
        public static final int NUM_MODULES = 4;
            // Module specific constants
        public static final class Mod0 {
            public static final int DRIVE_MOTOR_ID = 3;
            public static final int ANGLE_MOTOR_ID = 2;
            public static final int CANCODER_ID = 10;
            public static final double ANGLE_OFFSET = 105.03;
            public static final SwerveModuleConstants CONSTANTS =
                new SwerveModuleConstants(DRIVE_MOTOR_ID, ANGLE_MOTOR_ID, CANCODER_ID, ANGLE_OFFSET);
        }
        public static final class Mod1 {
            public static final int DRIVE_MOTOR_ID = 5;
            public static final int ANGLE_MOTOR_ID = 4;
            public static final int CANCODER_ID = 11;
            public static final double ANGLE_OFFSET = 334.95;
            public static final SwerveModuleConstants CONSTANTS =
                new SwerveModuleConstants(DRIVE_MOTOR_ID, ANGLE_MOTOR_ID, CANCODER_ID, ANGLE_OFFSET);
        }
        public static final class Mod2 {
            public static final int DRIVE_MOTOR_ID = 7;
            public static final int ANGLE_MOTOR_ID = 6;
            public static final int CANCODER_ID = 12;
            public static final double ANGLE_OFFSET = 34.10;
            public static final SwerveModuleConstants CONSTANTS =
                new SwerveModuleConstants(DRIVE_MOTOR_ID, ANGLE_MOTOR_ID, CANCODER_ID, ANGLE_OFFSET);
        }
        public static final class Mod3 {
            public static final int DRIVE_MOTOR_ID = 9;
            public static final int ANGLE_MOTOR_ID = 8;
            public static final int CANCODER_ID = 13;
            public static final double ANGLE_OFFSET = 306.47;
            public static final SwerveModuleConstants CONSTANTS =
                new SwerveModuleConstants(DRIVE_MOTOR_ID, ANGLE_MOTOR_ID, CANCODER_ID, ANGLE_OFFSET);
        }

        public static final int PIGEON = 30;

        public static final int SWERVE_MODULE_CURRENT_LIMIT = 60;
    }
    public static final class ArmConstants {
       public static final int ARM_MOTOR_ID = 10; 
       public static final int ARM_MOTOR_FOLLOW_ID = 11; 
       public static final int GRIPPER_SOLENOID_ID = 1;
       // public static final int CONE_SOLENOID_ID = 0;
       public enum Preset{
        HIGH_SCORE (20.0),
        MID_SCORE (0.0),
        LOW_SCORE (-10.0),
        HANDOFF (-40.0),
        STOWED (-80.0);
        private final double angle;
        Preset(double angle) {
        this.angle = angle;
        }
        public double angle() { return angle; }
       }
       public enum GripState{
        GRIP_OFF, GRIP_ON;
       }
       public static final double ARM_KP = 0;
       public static final double ARM_KI = 0;
       public static final double ARM_KD = 0;
       public static final double ARM_KS = 0;
       public static final double ARM_KG = 0;
       public static final double ARM_KV = 0;
       
       
    }
   

    public class OperatorConstants {
        public static final int DRIVER_CONTROLLER_PORT = 0;

        // Joystick Deadband
        public static final double LEFT_X_DEADBAND = 0.05;
        public static final double LEFT_Y_DEADBAND = 0.05;
    }

    public class Vision {
        public static final double POSE_ERROR_TOLERANCE = 0.5;
    }
}
