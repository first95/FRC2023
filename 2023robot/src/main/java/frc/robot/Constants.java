// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
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

    public static final int PNEUMATIC_HUB_ID = 60;
    public static final double KG_PER_LB = 0.453592;
    
    public static final double NEO_FREE_SPEED = 5676; // RPM
    public static final double NEO_STALL_TORQUE = 2.6; // N * m
    public static final double NEO_550_FREE_SPEED = 11000; // RPM
    
    public static final double ROBOT_MASS = (148 - 20.3) * KG_PER_LB; // 32lbs * kg per pound
    public static final double MANIPULATOR_MASS = 0 * 0.453592; // 10lbs * kg per pound
    public static final double CHASSIS_MASS = ROBOT_MASS - MANIPULATOR_MASS;
    public static final Translation3d CHASSIS_CG = new Translation3d(
        0,
        0,
        Units.inchesToMeters(8));
    public static final double ARM_Y_POS = 0; // centered on robot
    public static final double GRAVITY = 9.81; // m/s/s
    public static final double LOOP_TIME = 0.13; //s, 20ms + 110ms sprk max velocity lag

    public static final double dummyArmX = Units.inchesToMeters(42);
    public static final double dummyArmHieght = Units.inchesToMeters(27);

    public static final class Auton {
        public static final double X_KP = 0.7;
        public static final double X_KI = 0;
        public static final double X_KD = 0;

        public static final double Y_KP = 0.7;
        public static final double Y_KI = 0;
        public static final double Y_KD = 0;

        public static final double ANG_KP = 0.4;
        public static final double ANG_KI = 0;
        public static final double ANG_KD = 0.01;

        public static final double MAX_SPEED = 4;
        public static final double MAX_ACCELERATION = 2;
    }
    
    public static final class Drivebase {
        // Hold time on motor brakes when disabled
        public static final double WHEEL_LOCK_TIME = 10; // seconds

        public static final int SWERVE_MODULE_CURRENT_LIMIT = 60;

        // Robot heading control gains
        public static final double HEADING_KP = 0.4;
        public static final double HEADING_KI = 0;
        public static final double HEADING_KD = 0.01;

        // Motor and encoder inversions
        public static final boolean ABSOLUTE_ENCODER_INVERT = true;
        public static final boolean DRIVE_MOTOR_INVERT = true;
        public static final boolean ANGLE_MOTOR_INVERT = false;
        public static final boolean INVERT_GYRO = false;

        // Module locations, in meters, as distances to the center of the robot.
        // Positive x is torwards the robot front, and +y is torwards robot left.
        public static final double FRONT_LEFT_X = Units.inchesToMeters(10.25);
        public static final double FRONT_LEFT_Y = Units.inchesToMeters(10.25);
        public static final double FRONT_RIGHT_X = Units.inchesToMeters(10.25);
        public static final double FRONT_RIGHT_Y = Units.inchesToMeters(-10.25);
        public static final double BACK_LEFT_X = Units.inchesToMeters(-10.25);
        public static final double BACK_LEFT_Y = Units.inchesToMeters(10.25);
        public static final double BACK_RIGHT_X = Units.inchesToMeters(-10.25);
        public static final double BACK_RIGHT_Y = Units.inchesToMeters(-10.25);

        public static final Translation2d[] MODULE_LOCATIONS = {
            new Translation2d(Drivebase.FRONT_LEFT_X, Drivebase.FRONT_LEFT_Y),
            new Translation2d(Drivebase.FRONT_RIGHT_X, Drivebase.FRONT_RIGHT_Y),
            new Translation2d(Drivebase.BACK_LEFT_X, Drivebase.BACK_LEFT_Y),
            new Translation2d(Drivebase.BACK_RIGHT_X, Drivebase.BACK_RIGHT_Y)
        };

        // IMU Mounting. CCW Positive
        public static final double IMU_MOUNT_YAW = 0;
        public static final double IMU_MOUNT_PITCH = 0;
        public static final double IMU_MOUNT_ROLL = 0;

        // Drivetrain limitations
        public static final double MAX_SPEED = Units.feetToMeters(15.76); // meters per second
        public static final double MAX_ANGULAR_VELOCITY = MAX_SPEED / Math.hypot(FRONT_LEFT_X, FRONT_LEFT_Y); // rad/s
        // Theoretical max acceleration should be as follows:
        // (NEO stall torque * module gearing * number of modules) / (wheel radius * robot mass) = m/s/s
        // (2.6 * 6.75 * 4) / (Units.inchesToMeters(2) * ROBOT_MASS)
        // However, the drive is traction-limited, so the max accelration is actually (wheel coefficient of friction * gravity)
        public static final double MAX_ACCELERATION = 1.19 * 9.81; // COF (blue nitrile on carpet) as reported by Studica
        // max speed (RPM) / gear ratio, convert to deg/min, divide by 60 for deg/s
        public static final double MAX_MODULE_ANGULAR_SPEED = Units.rotationsToDegrees(NEO_550_FREE_SPEED * 7 / 372) / 60; // deg/s

        // Swerve base kinematics object
        public static final BetterSwerveKinematics KINEMATICS = new BetterSwerveKinematics(MODULE_LOCATIONS);

        // Module PIDF gains
        public static final double MODULE_KP = 0.01;
        public static final double MODULE_KI = 0;
        public static final double MODULE_KD = 0;
        public static final double MODULE_IZ = 0;
        public static final double MODULE_KF = 0;
        // Volt * seconds / degree.  Equal to (maxVolts) / ((degreesPerRotation) * (maxMotorSpeedRPM / gearRatio) * (minutesPerSecond))
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
        public static final double METERS_PER_MOTOR_ROTATION = (Math.PI * Units.inchesToMeters(3)) / 4.71;
            // Calculation: 3in diameter wheels * pi [circumfrence] / gear ratio
        public static final double DEGREES_PER_STEERING_ROTATION = 360;
            // degrees per rotation / gear ratio between module and motor
        
        public static final int NUM_MODULES = 4;
            // Module specific constants
        public static final class Mod0 {
            public static final int DRIVE_MOTOR_ID = 7;
            public static final int ANGLE_MOTOR_ID = 6;
            public static final double ANGLE_OFFSET = 111.9 + 90 - 161.4; // 231.48 + 90;
            public static final SwerveModuleConstants CONSTANTS =
                new SwerveModuleConstants(DRIVE_MOTOR_ID, ANGLE_MOTOR_ID, ANGLE_OFFSET);
        }
        public static final class Mod1 {
            public static final int DRIVE_MOTOR_ID = 9;
            public static final int ANGLE_MOTOR_ID = 8;
            public static final double ANGLE_OFFSET = 180.7 + 180; // 170.69 + 180;
            public static final SwerveModuleConstants CONSTANTS =
                new SwerveModuleConstants(DRIVE_MOTOR_ID, ANGLE_MOTOR_ID, ANGLE_OFFSET);
        }
        public static final class Mod2 {
            public static final int DRIVE_MOTOR_ID = 5;
            public static final int ANGLE_MOTOR_ID = 4;
            public static final double ANGLE_OFFSET = 52.3; // 52.67;
            public static final SwerveModuleConstants CONSTANTS =
                new SwerveModuleConstants(DRIVE_MOTOR_ID, ANGLE_MOTOR_ID, ANGLE_OFFSET);
        }
        public static final class Mod3 {
            public static final int DRIVE_MOTOR_ID = 3;
            public static final int ANGLE_MOTOR_ID = 2;
            public static final double ANGLE_OFFSET =  91 - 90 + 131 - 5; // 224.08 - 90;
            public static final SwerveModuleConstants CONSTANTS =
                new SwerveModuleConstants(DRIVE_MOTOR_ID, ANGLE_MOTOR_ID, ANGLE_OFFSET);
        }

        public static final int PIGEON = 30;
    }

    public class OperatorConstants {
        public static final int DRIVER_CONTROLLER_PORT = 0;

        // Joystick Deadband
        public static final double LEFT_X_DEADBAND = 0.05;
        public static final double LEFT_Y_DEADBAND = 0.05;

        // Joystick Deadband
        public static final double RIGHT_X_DEADBAND = 0.05;
        public static final double RIGHT_Y_DEADBAND = 0.1;
    }

    public static final class ArmConstants {
        public static final int ARM_MOTOR_ID = 10; 
        public static final int ARM_MOTOR_FOLLOWER_ID = 11; 
        public static final int GRIPPER_SOLENOID_ID = 1;

        // Limits in degrees
        public static final float ARM_LOWER_LIMIT = -10;
        public static final float ARM_UPPER_LIMIT = 160;

        // Ratio = 1 / 5 / 5 * 22 / 40
        // 5 to 1 | 5 to 1 | 40 to 22
        public static final double ARM_GEAR_RATIO = 0.022;
        public static final double ARM_DEGREES_PER_MOTOR_ROTATION = ARM_GEAR_RATIO * 360;

        public enum CONTROL_MODE {
            POSITION,
            DUTY,
            HOLD
          }

        // TODO: Change these presets to the correct angle!
        public enum PRESETS {
            HIGH_SCORE (122),
            MID_SCORE (105.7),
            LOW_SCORE (16),
            HANDOFF (13),
            STOWED (0);
            
            private final double angle;
            PRESETS(double angle) { this.angle = angle; }
            public double angle() { return angle; }
        }

        public enum GripState {GRIP_OFF, GRIP_ON}

        // Constants for PID
        public static final double ARM_KP = 0.008;
        public static final double ARM_KI = 0;
        public static final double ARM_KD = 0;
        public static final double ARM_KF = 0;

        // Constants for feed forward
        public static final double ARM_KS = 0;
        public static final double ARM_KG = 0;
        public static final double ARM_KV = 0;
     }

    public static final class IntakeConstants {
        public static final int BOTTOM_ROLLER_ID = 14;
        public static final int TOP_ROLLER_ID = 12;
        public static final int RACK_ID = 13;

        public static final boolean INVERT_ROLLERS = true;
        public static final boolean INVERT_RACK = false;

        public static final double PINION_PITCH_DIAMETER = Units.inchesToMeters(0.9);
        public static final double RACK_DRIVE_GEAR_RATIO = 0.2;
        public static final double INTAKE_MASS = 9 * KG_PER_LB;
        public static final double RACK_METERS_PER_MOTOR_ROTATION =
            (PINION_PITCH_DIAMETER * Math.PI) * RACK_DRIVE_GEAR_RATIO;
        public static final float RACK_LOWER_LIMIT = 0;
        public static final float RACK_UPPER_LIMIT = (float)Units.inchesToMeters(14);
        public static final double RACK_TRAVEL = RACK_UPPER_LIMIT - RACK_LOWER_LIMIT;

        public static final double KP = 10;
        public static final double KI = 0;
        public static final double KD = 0;
        public static final double KF = 0;
        public static final double IZ = 0;

        // motor max rpm --> m/s
        public static final double MAX_LINEAR_SPEED = 5676 * RACK_METERS_PER_MOTOR_ROTATION / 60;
        public static final double MAX_LINEAR_ACCELERATION =
            (NEO_STALL_TORQUE / RACK_DRIVE_GEAR_RATIO) * (PINION_PITCH_DIAMETER / 2) / INTAKE_MASS;

        public static final double KV = 12 / MAX_LINEAR_SPEED;
        public static final double KA = 12 / MAX_LINEAR_ACCELERATION;

        public enum PRESETS {
            STOWED (0),
            HANDOFF (0),
            CONE (0.15),
            CUBE (Units.inchesToMeters(13.5));

            private final double position;
            PRESETS(double position) {
                this.position = position;
            }

            public double position() {
                return position;
            }
        }
    }
}
