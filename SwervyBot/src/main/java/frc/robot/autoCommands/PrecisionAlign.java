// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autoCommands;

import frc.robot.Constants.Auton;
import frc.robot.subsystems.SwerveBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class PrecisionAlign extends CommandBase {
  private final SwerveBase drive;
  private final Pose2d target;
  private final PIDController xController, yController, angController;

  private Pose2d currentPose;
  private double xVel, yVel, omega;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public PrecisionAlign(String pose, Alliance alliance, SwerveBase drive) {
    this.drive = drive;
    this.target = Auton.POSE_MAP.get(alliance).get(pose);
    xController = new PIDController(Auton.X_KP, Auton.X_KI, Auton.X_KD);
    yController = new PIDController(Auton.Y_KP, Auton.Y_KI, Auton.Y_KD);
    angController = new PIDController(Auton.ANG_KP, Auton.ANG_KI, Auton.ANG_KD);

    angController.enableContinuousInput(-Math.PI, Math.PI);
    
    xController.setTolerance(Auton.X_TOLERANCE);
    yController.setTolerance(Auton.Y_TOLERANCE);
    angController.setTolerance(Auton.ANG_TOLERANCE);
 
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    xController.setSetpoint(target.getX());
    yController.setSetpoint(target.getY());
    angController.setSetpoint(target.getRotation().getRadians());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentPose = drive.getPose();
    xVel = xController.calculate(currentPose.getX());
    yVel = yController.calculate(currentPose.getY());
    omega = angController.calculate(currentPose.getRotation().getRadians());
    drive.drive(new Translation2d(xVel, yVel), omega, true, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.drive(new Translation2d(), 0, true, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (xController.atSetpoint() && yController.atSetpoint() && angController.atSetpoint());
  }
}
