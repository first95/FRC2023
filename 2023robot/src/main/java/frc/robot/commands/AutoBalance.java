// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SwerveBase;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutoBalance extends CommandBase {
  private final SwerveBase m_SwerveBase;

  private double error;
  private double currentAngle;
  private double driveVelocity;

  public AutoBalance(SwerveBase swerveBase) {
    m_SwerveBase = swerveBase;
    addRequirements(swerveBase);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Double simAngle = -1 * RobotContainer.weaponsController.getRawAxis(1) * 45;
    this.currentAngle = simAngle;
    // this.currentAngle = m_SwerveBase.getPitch();

    error = Constants.CHARGER_GOAL_DEGREES - currentAngle;
    driveVelocity = -Math.min(Constants.CHARGER_KP * error, 5);

    /* This command will change depending on: In robot-relative mode, positive x is torwards the bow (front) 
     * and positive y is torwards port (left).  In field-relative mode, positive x is away from the alliance 
     * wall (field North) and positive y is torwards the left wall when looking 
     * through the driver station glass (field West). */
    m_SwerveBase.drive(
      new Translation2d(0, driveVelocity),
      0,
      false,
      false
    );
    
    // Debugging Print Statments
    System.out.println("Current Angle: " + currentAngle);
    System.out.println("Error " + error);
    System.out.println("Drive Velocity: " + driveVelocity);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(error) < Constants.CHARGER_THRESHOLD_DEGREES;
  }
}
