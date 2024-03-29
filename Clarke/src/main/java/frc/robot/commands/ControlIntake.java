// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Intake;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class ControlIntake extends CommandBase {
  private final Intake subsystem;
  private DoubleSupplier coneSpeed, cubeSpeed, position;
  private BooleanSupplier setStowed, setHandoff, setCone, setCube;
  private boolean grabbedCone;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ControlIntake(DoubleSupplier coneSpeed, DoubleSupplier cubeSpeed, DoubleSupplier position, BooleanSupplier setStowed, BooleanSupplier setHandoff, BooleanSupplier setCone, BooleanSupplier setCube, Intake subsystem) {
    this.subsystem = subsystem;
    this.coneSpeed = coneSpeed;
    this.cubeSpeed = cubeSpeed;
    this.setStowed = setStowed;
    this.setHandoff = setHandoff;
    this.setCone = setCone;
    this.setCube = setCube;
    this.position = position;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //subsystem.runRollers(coneSpeed.getAsDouble(), cubeSpeed.getAsDouble());
    //subsystem.moveIntake(position.getAsDouble());
    //subsystem.setPosition(IntakeConstants.RACK_TRAVEL * position.getAsDouble());
    if (setStowed.getAsBoolean()) {
      subsystem.setPreset(IntakeConstants.PRESETS.STOWED);
      subsystem.runRollers(0, 0);
      grabbedCone = false;
    } 
    else if (setHandoff.getAsBoolean()) {
      subsystem.setPreset(IntakeConstants.PRESETS.HANDOFF);
      subsystem.grabCone(-0.6);
      grabbedCone = false;
    } 
    else if (setCone.getAsBoolean()) {
      subsystem.setPreset(IntakeConstants.PRESETS.CONE);
      if(subsystem.rackHasReachedReference(IntakeConstants.PRESETS.CONE.position()))
        subsystem.grabCone(0.6);
      grabbedCone = true;
      SmartDashboard.putString("LastHandoff", "CONE");
    } 
    else if (setCube.getAsBoolean()) {
      subsystem.setPreset(IntakeConstants.PRESETS.CUBE);
      if(subsystem.rackHasReachedReference(IntakeConstants.PRESETS.CUBE.position()))
        subsystem.grabCube(0.6);
      grabbedCone = false;
      SmartDashboard.putString("LastHandoff", "CUBE");
    } else if (grabbedCone){
      subsystem.setPreset(IntakeConstants.PRESETS.STOWED);
      subsystem.grabCone(0.08);
      if (SmartDashboard.getBoolean("ConeHandoffOccurred", false)) {
        grabbedCone = false;
        SmartDashboard.putBoolean("ConeHandoffOccurred", false);
      }
    } else {
      subsystem.setPreset(IntakeConstants.PRESETS.STOWED);
      subsystem.runRollers(0, 0);
    }
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
