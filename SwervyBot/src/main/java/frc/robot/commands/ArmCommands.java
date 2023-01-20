// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Arm;
import frc.robot.subsystems.SwerveBase;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class ArmCommands extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Arm a_sub;
  private final SwerveBase d_sub;

  public ArmCommands(Arm subsystemA, SwerveBase subsystemD) {
    a_sub = subsystemA;
    d_sub = subsystemD;
    addRequirements(a_sub);
    addRequirements(d_sub);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    a_sub.setPos(0.8);
    d_sub.driveBack(backwards);
    a_sub.setPos(0.5);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
