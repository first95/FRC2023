// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.SwerveModule;
import frc.robot.Constants.Drivebase;

public class SwerveBase extends SubsystemBase {

  private SwerveModule[] swerveModules;
  private PigeonIMU imu;
  
  private SwerveDriveOdometry odometry;
  public Field2d field = new Field2d();

  private double angle, lasttime;

  private Timer timer, disabledTimer;

  private boolean wasGyroReset;

  /** Creates a new swerve drivebase subsystem.  Robot is controlled via the drive() method,
   * or via the setModuleStates() method.  The drive() method incorporates kinematics— it takes a 
   * translation and rotation, as well as parameters for field-centric and closed-loop velocity control.
   * setModuleStates() takes a list of SwerveModuleStates and directly passes them to the modules.
   * This subsytem also handles odometry.
  */
  public SwerveBase() {

    // Create an integrator for angle if the robot is being simulated to emulate an IMU
    // If the robot is real, instantiate the IMU instead.
    if (!Robot.isReal()) {
      timer = new Timer();
      timer.start();
      lasttime = 0;
    } else {
      imu = new PigeonIMU(Drivebase.PIGEON);
      imu.configFactoryDefault();
      zeroGyro();
    }

    odometry = new SwerveDriveOdometry(Drivebase.KINEMATICS, getYaw(), getModulePositions());

    swerveModules = new SwerveModule[] {
      new SwerveModule(0, Drivebase.Mod0.CONSTANTS),
      new SwerveModule(1, Drivebase.Mod1.CONSTANTS),
      new SwerveModule(2, Drivebase.Mod2.CONSTANTS),
      new SwerveModule(3, Drivebase.Mod3.CONSTANTS),
    };
  }

  /**
   * The primary method for controlling the drivebase.  Takes a Translation2d and a rotation rate, and 
   * calculates and commands module states accordingly.  Can use either open-loop or closed-loop velocity
   * control for the wheel velocities.  Also has field- and robot-relative modes, which affect how the translation
   *  vector is used.
   * @param translation  Translation2d that is the commanded linear velocity of the robot, in meters per second.
   * In robot-relative mode, positive x is torwards the bow (front) and positive y is torwards port (left).  In field-
   * relative mode, positive x is away from the alliance wall (field North) and positive y is torwards the left wall when looking 
   * through the driver station glass (field West).
   * @param rotation  Robot angular rate, in radians per second. CCW positive.  Unaffected by field/robot relativity.
   * @param fieldRelative  Drive mode.  True for field-relative, false for robot-relative.
   * @param isOpenLoop  Whether or not to use closed-loop velocity control.  Set to true to disable closed-loop.
   */
  public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
    // Creates a robot-relative ChassisSpeeds object, converting from field-relative speeds if necessary.
    ChassisSpeeds velocity = fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
      translation.getX(), 
      translation.getY(), 
      rotation, 
      getYaw()
    )
    : new ChassisSpeeds(
      translation.getX(),
      translation.getY(),
      rotation
    );

    // Display commanded speed for testing
    SmartDashboard.putString("RobotVelocity", velocity.toString());

    // Calculate required module states via kinematics
    SwerveModuleState[] swerveModuleStates = 
      Drivebase.KINEMATICS.toSwerveModuleStates(
        velocity
      );
    // Desaturate calculated speeds
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Drivebase.MAX_SPEED);

    // Command and display desired states
    for (SwerveModule module : swerveModules) {
      SmartDashboard.putString("Module" + module.toString(), swerveModuleStates[module.moduleNumber].toString());
      module.setDesiredState(swerveModuleStates[module.moduleNumber], isOpenLoop);
    }
  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Drivebase.MAX_SPEED);

    for (SwerveModule module : swerveModules) {
      module.setDesiredState(desiredStates[module.moduleNumber], false);
    }
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose) {
    odometry.resetPosition(getYaw(), getModulePositions(), pose);
  }

  public SwerveModuleState[] getStates() {
    SwerveModuleState[] states = new SwerveModuleState[Drivebase.NUM_MODULES];
    for (SwerveModule module : swerveModules) {
      states[module.moduleNumber] = module.getState();
    }
    return states;
  }

  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[Drivebase.NUM_MODULES];
    for (SwerveModule module : swerveModules) {
      positions[module.moduleNumber] = module.getPosition();
    }
    return positions;
  }

  public boolean wasGyroReset() {
    return wasGyroReset;
  }

  public void clearGyroReset() {
    wasGyroReset = false;
  }

  public void zeroGyro() {
    if (Robot.isReal()) {
      imu.setYaw(0);
    } else {
      angle = 0;
    }
    wasGyroReset = true;
  }

  public Rotation2d getYaw() {
    if (Robot.isReal()) {
      double[] ypr = new double[3];
      imu.getYawPitchRoll(ypr);
      return (Drivebase.INVERT_GYRO) ? Rotation2d.fromDegrees(360 - ypr[0]) : Rotation2d.fromDegrees(ypr[0]);
    } else {
      return new Rotation2d(angle);
    }
  }

  public void setMotorBrake(boolean brake) {
    for (SwerveModule swerveModule : swerveModules) {
      swerveModule.setMotorBrake(brake);
    }
  }

  public void setDriveBrake() {
    for (SwerveModule swerveModule : swerveModules) {
      swerveModule.setDesiredState(
        new SwerveModuleState(
          0,
          Drivebase.MODULE_LOCATIONS[swerveModule.moduleNumber].getAngle()),
        true);
    }
  }

  @Override
  public void periodic() {
    odometry.update(getYaw(), getModulePositions());

    if (!Robot.isReal()) {
      angle += Drivebase.KINEMATICS.toChassisSpeeds(getStates()).omegaRadiansPerSecond * (timer.get() - lasttime);
      lasttime = timer.get();

      field.setRobotPose(odometry.getPoseMeters());
      SmartDashboard.putData("Field", field);
    }

    for (SwerveModule module : swerveModules) {
      SmartDashboard.putNumber("Module" + module.moduleNumber + "CANCoder", module.getCANCoder());
      SmartDashboard.putNumber("Module" + module.moduleNumber + "Relative Encoder", module.getRelativeEncoder());
    }
  }

  @Override
  public void simulationPeriodic() {
  }
}
