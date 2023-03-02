// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;
import com.ctre.phoenix.sensors.Pigeon2Configuration;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.BetterSwerveKinematics;
import frc.lib.util.BetterSwerveModuleState;
import frc.robot.Robot;
import frc.robot.SwerveModule;
import frc.robot.Constants.Drivebase;
import frc.robot.Constants.Vision;

public class SwerveBase extends SubsystemBase {

  private SwerveModule[] swerveModules;
  private Pigeon2 imu;
  private NetworkTable portLimelightData, starboardLimelightData;
  private boolean wasOdometrySeeded;
  
  private SwerveDrivePoseEstimator odometry;
  public Field2d field = new Field2d();

  private double angle, lasttime, visionLatency;

  private Timer timer;

  private boolean wasGyroReset;

  private Alliance alliance = Alliance.Invalid;

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
      imu = new Pigeon2(Drivebase.PIGEON);
      imu.configFactoryDefault();
      Pigeon2Configuration config = new Pigeon2Configuration();
      config.MountPosePitch = Drivebase.IMU_MOUNT_PITCH;
      config.MountPoseRoll = Drivebase.IMU_MOUNT_ROLL;
      config.MountPoseYaw = Drivebase.IMU_MOUNT_YAW;
      imu.configAllSettings(config);
    }

    this.swerveModules = new SwerveModule[] {
      new SwerveModule(0, Drivebase.Mod0.CONSTANTS),
      new SwerveModule(1, Drivebase.Mod1.CONSTANTS),
      new SwerveModule(2, Drivebase.Mod2.CONSTANTS),
      new SwerveModule(3, Drivebase.Mod3.CONSTANTS),
    };

    portLimelightData = NetworkTableInstance.getDefault().getTable("limelight-" + Vision.PORT_LIMELIGHT_NAME);
    starboardLimelightData = NetworkTableInstance.getDefault().getTable("limelight-" + Vision.STARBOARD_LIMELIGHT_NAME);

    odometry = new SwerveDrivePoseEstimator(Drivebase.KINEMATICS, getYaw(), getModulePositions(), new Pose2d());
    wasOdometrySeeded = false;
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
    BetterSwerveModuleState[] swerveModuleStates = 
      Drivebase.KINEMATICS.toSwerveModuleStates(
        velocity
      );
    
    // Desaturate calculated speeds
    BetterSwerveKinematics.desaturateWheelSpeeds(swerveModuleStates, Drivebase.MAX_SPEED);

    // Command and display desired states
    for (SwerveModule module : swerveModules) {
      SmartDashboard.putNumber("Module " + module.moduleNumber + " Speed Setpoint: ", swerveModuleStates[module.moduleNumber].speedMetersPerSecond);
      SmartDashboard.putNumber("Module " + module.moduleNumber + " Angle Setpoint: ", swerveModuleStates[module.moduleNumber].angle.getDegrees());
      module.setDesiredState(swerveModuleStates[module.moduleNumber], isOpenLoop);
    }
  }

  /**
   * Set the module states (azimuth and velocity) directly.  Used primarily for auto
   * pathing.
   * @param desiredStates  A list of SwerveModuleStates to send to the modules.
   */
  public void setModuleStates(BetterSwerveModuleState[] desiredStates) {
    // Desaturates wheel speeds
    BetterSwerveKinematics.desaturateWheelSpeeds(desiredStates, Drivebase.MAX_SPEED);

    // Sets states
    for (SwerveModule module : swerveModules) {
      module.setDesiredState(desiredStates[module.moduleNumber], false);
    }
  }

  /**
   * Set robot-relative chassis speeds with closed-loop velocity control.
   * @param chassisSpeeds Robot-relative.
   */
  public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
    setModuleStates(
      Drivebase.KINEMATICS.toSwerveModuleStates(chassisSpeeds));
  }

  
  /**
   * Gets the current pose (position and rotation) of the robot, as reported by odometry.
   * @return The robot's pose
   */
  public Pose2d getPose() {
    return odometry.getEstimatedPosition();
  }

  /**
   * Gets the current field-relative velocity (x, y and omega) of the robot
   * @return A ChassisSpeeds object of the current field-relative velocity
   */
  public ChassisSpeeds getFieldVelocity() {
    // ChassisSpeeds has a method to convert from field-relative to robot-relative speeds,
    // but not the reverse.  However, because this transform is a simple rotation, negating the angle
    // given as the robot angle reverses the direction of rotation, and the conversion is reversed.
    return ChassisSpeeds.fromFieldRelativeSpeeds(Drivebase.KINEMATICS.toChassisSpeeds(getStates()), getYaw().unaryMinus());
  }

  /**
   * Gets the current robot-relative velocity (x, y and omega) of the robot
   * @return A ChassisSpeeds object of the current robot-relative velocity
   */
  public ChassisSpeeds getRobotVelocity() {
    return Drivebase.KINEMATICS.toChassisSpeeds(getStates());
  }

  
  /**
   * Resets odometry to the given pose. Gyro angle and module positions do not need to 
   * be reset when calling this method.  However, if either gyro angle or module position
   * is reset, this must be called in order for odometry to keep working.
   * @param pose The pose to set the odometry to
   */
  public void resetOdometry(Pose2d pose) {
    odometry.resetPosition(getYaw(), getModulePositions(), pose);
  }

  /**
   * Gets the current module states (azimuth and velocity)
   * @return A list of SwerveModuleStates containing the current module states
   */
  public BetterSwerveModuleState[] getStates() {
    BetterSwerveModuleState[] states = new BetterSwerveModuleState[Drivebase.NUM_MODULES];
    for (SwerveModule module : swerveModules) {
      states[module.moduleNumber] = module.getState();
    }
    return states;
  }

  /**
   * Gets the current module positions (azimuth and wheel position (meters))
   * @return A list of SwerveModulePositions cointaing the current module positions
   */
  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[Drivebase.NUM_MODULES];
    for (SwerveModule module : swerveModules) {
      positions[module.moduleNumber] = module.getPosition();
    }
    return positions;
  }

  /**
   * A public method to allow other systems to determine if the gyro was reset by accessing
   * the wasGyroReset flag.
   * @return The boolean value of wasGyroReset
   */
  public boolean wasGyroReset() {
    return wasGyroReset;
  }

  /**
   * Sets wasGyroReset to false.  Should be called after all systems that need to know have called
   * wasGyroReset.
   */
  public void clearGyroReset() {
    wasGyroReset = false;
  }

  /**
   * Resets the gyro angle to zero and resets odometry to the same position, but facing toward 0.
   * Also sets the wasGyroReset flag to true.
   */
  public void zeroGyro() {
    // Resets the real gyro or the angle accumulator, depending on whether the robot is being simulated
    if (Robot.isReal()) {
      imu.setYaw(0);
    } else {
      angle = 0;
    }
    wasGyroReset = true;
    resetOdometry(new Pose2d(getPose().getTranslation(), new Rotation2d()));
  }

  public void setGyro(Rotation2d angle) {
    // Resets the real gyro or the angle accumulator, depending on whether the robot is being simulated
    if (Robot.isReal()) {
      imu.setYaw(angle.getDegrees());
    } else {
      this.angle = angle.getRadians();
    }
    wasGyroReset = true;
    resetOdometry(new Pose2d(getPose().getTranslation(), angle));
  }

  /**
   * Gets the current yaw angle of the robot, as reported by the imu.  CCW positive, not wrapped.
   * @return The yaw angle
   */
  public Rotation2d getYaw() {
    // Read the imu if the robot is real or the accumulator if the robot is simulated.
    if (Robot.isReal()) {
      double yaw = imu.getYaw();
      return (Drivebase.INVERT_GYRO) ? Rotation2d.fromDegrees(360 - yaw) : Rotation2d.fromDegrees(yaw);
    } else {
      return new Rotation2d(angle);
    }
  }

  public Rotation2d getPitch() {
    return Rotation2d.fromDegrees(imu.getPitch());
  }

  public Rotation2d getRoll() {
    return Rotation2d.fromDegrees(imu.getRoll());
  }

  /**
   * Sets the drive motors to brake/coast mode.
   * @param brake True to set motors to brake mode, false for coast.
   */
  public void setMotorBrake(boolean brake) {
    for (SwerveModule swerveModule : swerveModules) {
      swerveModule.setMotorBrake(brake);
    }
  }

  /**
   * Point all modules toward the robot center, thus making the robot very difficult to move.
   */
  public void setDriveBrake() {
    for (SwerveModule swerveModule : swerveModules) {
      swerveModule.setDesiredState(
        new BetterSwerveModuleState(
          0,
          Drivebase.MODULE_LOCATIONS[swerveModule.moduleNumber].getAngle(),
          0),
        true,
        false);
    }
  }

  public void setAlliance(Alliance alliance) {
    this.alliance = alliance;
    wasOdometrySeeded = false;
  }

  public Pose3d getVisionPose(NetworkTable visionData) {
    if ((visionData.getEntry("tv").getDouble(0) == 0 ||
      visionData.getEntry("getPipe").getDouble(0) != Vision.APRILTAG_PIPELINE_NUMBER)) {
      DriverStation.reportWarning("Limelight no target" + visionData.getPath(), false);
      return null;
    }
    double[] poseComponents;
    if (alliance == Alliance.Blue) {
      poseComponents = visionData.getEntry("botpose_wpiblue").getDoubleArray(new double[7]);
    } else if (alliance == Alliance.Red) {
      poseComponents = visionData.getEntry("botpose_wpired").getDoubleArray(new double[7]);
    } else {
      return null;
    }
    visionLatency = poseComponents[6];
    return new Pose3d(
        poseComponents[0],
        poseComponents[1],
        poseComponents[2],
        new Rotation3d(
          Math.toRadians(poseComponents[3]),
          Math.toRadians(poseComponents[4]),
          Math.toRadians(poseComponents[5])));
  }

  /*public void setVelocityModuleGains() {
    for (SwerveModule swerveModule : swerveModules) {
      swerveModule.setGains(
        SmartDashboard.getNumber("KP", Drivebase.VELOCITY_KP),
        SmartDashboard.getNumber("KI", Drivebase.VELOCITY_KI),
        SmartDashboard.getNumber("KD", Drivebase.VELOCITY_KD),
        SmartDashboard.getNumber("KS", Drivebase.KS),
        SmartDashboard.getNumber("KV", Drivebase.KV),
        SmartDashboard.getNumber("KA", Drivebase.KA));
    }
  }*/

  @Override
  public void periodic() {
    SmartDashboard.putString("Gyro", getYaw().toString());
    SmartDashboard.putString("alliance", alliance.toString());
    SmartDashboard.putBoolean("seeded", wasOdometrySeeded);
    // Seed odometry if this has not been done
    if (!wasOdometrySeeded) { 
      Pose3d portSeed3d = getVisionPose(portLimelightData);
      Pose3d starboardSeed3d = getVisionPose(starboardLimelightData);
      if ((portSeed3d != null) && (starboardSeed3d != null)) {
        // Average position, pick a camera for rotation
        Pose2d portSeed = portSeed3d.toPose2d();
        Pose2d starboardSeed = starboardSeed3d.toPose2d();
        Translation2d translation = portSeed.getTranslation().plus(starboardSeed.getTranslation()).div(2);
        //Rotation2d rotation = starboardSeed.getRotation();
        // Rotation average:
        Rotation2d rotation = 
          new Translation2d(
            (portSeed.getRotation().getCos() + starboardSeed.getRotation().getCos()) / 2,
            (portSeed.getRotation().getSin() + starboardSeed.getRotation().getSin()) / 2)
          .getAngle();
        imu.setYaw(rotation.getDegrees());
        resetOdometry(new Pose2d(translation, rotation));
        wasOdometrySeeded = true;
        wasGyroReset = true;
      }
      else if (starboardSeed3d != null) {
        Pose2d starboardSeed = new Pose2d(
          new Translation2d(
            starboardSeed3d.getX(),
            starboardSeed3d.getY()),
          new Rotation2d(starboardSeed3d.getRotation().getZ()));
        imu.setYaw(starboardSeed.getRotation().getDegrees());
        resetOdometry(starboardSeed);
        wasOdometrySeeded = true;
        wasGyroReset = true;
      }
      else if (portSeed3d != null) {
        Pose2d portSeed = new Pose2d(
          new Translation2d(
            portSeed3d.getX(),
            portSeed3d.getY()),
          new Rotation2d(portSeed3d.getRotation().getZ()));
        imu.setYaw(portSeed.getRotation().getDegrees());
        resetOdometry(portSeed);
        wasOdometrySeeded = true;
        wasGyroReset = true;
      } else {
        DriverStation.reportWarning("Alliance not set or tag not visible", false);
      }
    }
    
    // Update odometry
    odometry.update(getYaw(), getModulePositions());
    double timestamp;
    Pose3d portPose3d = getVisionPose(portLimelightData);
    double portTime = visionLatency;
    if (portPose3d != null) {
      Pose2d portPose = portPose3d.toPose2d();
      if (portPose.minus(getPose()).getTranslation().getNorm() <= Vision.POSE_ERROR_TOLERANCE) {
        timestamp = Timer.getFPGATimestamp() - portTime / 1000;
        odometry.addVisionMeasurement(portPose, timestamp);
      }
    }
    Pose3d starboardPose3d = getVisionPose(starboardLimelightData);
    double starboardTime = visionLatency;
    if (starboardPose3d != null) {
      Pose2d starboardPose = starboardPose3d.toPose2d();
      if (starboardPose.minus(getPose()).getTranslation().getNorm() <= Vision.POSE_ERROR_TOLERANCE) {
        timestamp = Timer.getFPGATimestamp() - starboardTime / 1000;
        odometry.addVisionMeasurement(starboardPose, timestamp);
      }
    }

    /*ChassisSpeeds robotVelocity = getRobotVelocity();
    SmartDashboard.putNumber("Robot X Vel", robotVelocity.vxMetersPerSecond);
    SmartDashboard.putNumber("Robot Y Vel", robotVelocity.vyMetersPerSecond);
    SmartDashboard.putNumber("Robot Ang Vel", robotVelocity.omegaRadiansPerSecond);*/

    Pose2d pose = odometry.getEstimatedPosition();
    SmartDashboard.putNumber("Robot X Vel", pose.getRotation().getDegrees());
    SmartDashboard.putString("Odometry", pose.toString());
    // Update angle accumulator if the robot is simulated
    if (!Robot.isReal()) {
      angle += Drivebase.KINEMATICS.toChassisSpeeds(getStates()).omegaRadiansPerSecond * (timer.get() - lasttime);
      lasttime = timer.get();

      field.setRobotPose(odometry.getEstimatedPosition());
      SmartDashboard.putData("Field", field);
    }

    double[] moduleStates = new double[8];
    for (SwerveModule module : swerveModules) {
      SmartDashboard.putNumber("Module" + module.moduleNumber + "CANCoder", module.getAbsoluteEncoder());
      moduleStates[module.moduleNumber] = module.getState().angle.getDegrees();
      moduleStates[module.moduleNumber + 1] = module.getState().speedMetersPerSecond;
    }
    SmartDashboard.putNumberArray("moduleStates", moduleStates);
  }

  @Override
  public void simulationPeriodic() {
  }


  public void turnModules(double speed) {
    for (SwerveModule swerveModule : swerveModules) {
      swerveModule.turnModule(speed);
    }
  }
}
