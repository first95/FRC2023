// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.drivebase;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.Drivebase;
import frc.robot.subsystems.SwerveBase;

/** An example command that uses an example subsystem. */
public class AbsoluteDrive extends CommandBase {
  private SwerveBase swerve;
  private PIDController thetaController;
  private DoubleSupplier vX, vY, headingHorizontal, headingVertical;
  private double omega, angle, lastAngle, x, y;
  private boolean isOpenLoop;

  /**
   * Used to drive a swerve robot in full field-centric mode.  vX and vY supply 
   * translation inputs, where x is torwards/away from alliance wall and y is left/right.
   * headingHorzontal and headingVertical are the Cartesian coordinates from which the robot's angle
   * will be derived— they will be converted to a polar angle, which the robot will rotate to.
   *
   * @param swerve The swerve drivebase subsystem.
   * @param vX DoubleSupplier that supplies the x-translation joystick input.  Should
   * be in the range -1 to 1 with deadband already accounted for.  Positive X is away from the alliance
   * wall.
   * @param vY DoubleSupplier that supplies the y-translation joystick input.  Should be in the
   * range -1 to 1 with deadband already accounted for.  Positive Y is towards the left wall when looking
   * through the driver station glass.
   * @param headingHorizontal DoubleSupplier that supplies the horizontal component of the robot's heading angle.
   * In the robot coordinate system, this is along the same axis as vY. Should range from -1 to 1 with no 
   * deadband.  Positive is towards the left wall when looking through the driver station glass.
   * @param headingVertical DoubleSupplier that supplies the vertical component of the robot's heading angle.  In the
   * robot coordinate system, this is along the same axis as vX.  Should range from -1 to 1 with no deadband.
   * Positive is away from the alliance wall.
   */
  public AbsoluteDrive(SwerveBase swerve, DoubleSupplier vX, DoubleSupplier vY, DoubleSupplier headingHorizontal, DoubleSupplier headingVertical, boolean isOpenLoop) {
    this.swerve = swerve;
    this.vX = vX;
    this.vY = vY;
    this.headingHorizontal = headingHorizontal;
    this.headingVertical = headingVertical;
    this.isOpenLoop = isOpenLoop;
    
    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    thetaController = new PIDController(Drivebase.HEADING_KP, Drivebase.HEADING_KI, Drivebase.HEADING_KD);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    lastAngle = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Checks if the gyro was reset, and, if so, sets the commanded heading to zero.
    // This allows the field refrence frame (which way is away from the alliance wall) to be
    // reset without the robot immediately rotating to the previously-commanded angle in the new
    // refrence frame.  This currently does not override the joystick.
    if (swerve.wasGyroReset()) {
      lastAngle = 0;
      swerve.clearGyroReset();
    }

    // Converts the horizontal and vertical components to the commanded angle, in radians, unless
    // the joystick is near the center (i. e. has been released), in which case the angle is held
    // at the last valid joystick input (hold position when stick released).
    if (Math.hypot(headingHorizontal.getAsDouble(), headingVertical.getAsDouble()) < 0.5) {
      angle = lastAngle;
    } else {
      angle = Math.atan2(headingHorizontal.getAsDouble(), headingVertical.getAsDouble());
    }
    // Calculates an angular rate using a PIDController and the commanded angle.  This is then scaled by
    // the drivebase's maximum angular velocity.
    omega = thetaController.calculate(swerve.getYaw().getRadians(), angle) * Drivebase.MAX_ANGULAR_VELOCITY;
    // Convert joystick inputs to m/s by scaling by max linear speed.  Also uses a cubic function
    // to allow for precise control and fast movement.
    x = Math.pow(vX.getAsDouble(), 3) * Drivebase.MAX_SPEED;
    y = Math.pow(vY.getAsDouble(), 3) * Drivebase.MAX_SPEED;

    // Make the robot move
    swerve.drive(new Translation2d(x, y), omega, true, isOpenLoop);
    
    // Used for the position hold feature
    lastAngle = angle;
  }

  private double calcMaxAccel(Translation2d commandedVelocity) {
    //remember to special-case sideways
    double armHeight = SmartDashboard.getNumber("armHeight", 0);
    double armExtension = SmartDashboard.getNumber("armExtension", 0);
    double verticalMoment = 
      (armHeight * Constants.MANIPULATOR_MASS)
      + (Constants.CHASSIS_CG.getZ() * (Constants.CHASSIS_MASS));
    Rotation2d angle = commandedVelocity.getAngle();

    Translation2d projectedChassisCg = new Translation2d(
      (angle.getSin() * angle.getCos() * Constants.CHASSIS_CG.getY()) + (Math.pow(angle.getCos(), 2) * Constants.CHASSIS_CG.getX()),
      (angle.getSin() * angle.getCos() * Constants.CHASSIS_CG.getX()) + (Math.pow(angle.getSin(), 2) * Constants.CHASSIS_CG.getY())
    );
    Translation2d projectedManipulatorCg = new Translation2d(
      (angle.getSin() * angle.getCos() * Constants.ARM_Y_POS) + (Math.pow(angle.getCos(), 2) * armExtension),
      (angle.getSin() * angle.getCos() * armExtension) + (Math.pow(angle.getSin(), 2) * Constants.ARM_Y_POS)
    );
    Translation2d projectedWheelbaseEdge;
    double angDeg = angle.getDegrees();
    if (angDeg <= 45 && angDeg >= -45) {
      projectedWheelbaseEdge = new Translation2d(
        Drivebase.FRONT_LEFT_X,
        Drivebase.FRONT_LEFT_X * angle.getTan());
    } else if (135 >= angDeg && angDeg > 45) {
      projectedWheelbaseEdge = new Translation2d(
        Drivebase.FRONT_LEFT_Y / angle.getTan(),
        Drivebase.FRONT_LEFT_Y);
    } else if (-135 <= angDeg && angDeg < -45) {
      projectedWheelbaseEdge = new Translation2d(
        Drivebase.FRONT_RIGHT_Y / angle.getTan(),
        Drivebase.FRONT_RIGHT_Y);
    } else {
      projectedWheelbaseEdge = new Translation2d(
        Drivebase.BACK_LEFT_X,
        Drivebase.BACK_LEFT_X * angle.getTan());
    }

    double directionalMoment = 
      projectedManipulatorCg.minus(projectedWheelbaseEdge)
        .times(Constants.MANIPULATOR_MASS)
        .plus(projectedChassisCg.minus(projectedWheelbaseEdge)
          .times(Constants.CHASSIS_MASS)).getNorm();

    double maxAccel = (Constants.GRAVITY * directionalMoment) / verticalMoment;
    return maxAccel;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
