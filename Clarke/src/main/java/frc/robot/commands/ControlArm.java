// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.Arm;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ControlArm extends CommandBase {
  private Arm arm;
  private DoubleSupplier manualControl;
  private BooleanSupplier setStowed, setHandoff, setHighScore, setMedScore, setLowScore;
  private boolean wasVelocityControlled;
  
  public ControlArm(
      DoubleSupplier manualControl, 
      BooleanSupplier setStowed, 
      BooleanSupplier setMedScore, 
      BooleanSupplier setLowScore, 
      BooleanSupplier setHighScore,
      BooleanSupplier setHandoff, 
      Arm arm
    ) {
    this.arm = arm;

    this.manualControl = manualControl;
    this.setStowed = setStowed;
    this.setHandoff = setHandoff;
    this.setHighScore = setHighScore;
    this.setMedScore = setMedScore;
    this.setLowScore = setLowScore;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(arm);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    double velocity = manualControl.getAsDouble() * ArmConstants.ARM_SPEED_LIMIT_RAD_PER_S;
    if (velocity != 0) {
      //arm.setVelocity(velocity);
      wasVelocityControlled = true;
    } else {
      if (setStowed.getAsBoolean()) {
        arm.setPreset(ArmConstants.PRESETS.STOWED);
        arm.setGrip(true);
      } else if (setHandoff.getAsBoolean()) {
        arm.setPreset(ArmConstants.PRESETS.HANDOFF);
      } else if (setLowScore.getAsBoolean()) {
        arm.setPreset(ArmConstants.PRESETS.LOW_SCORE);
      } else if (setMedScore.getAsBoolean()) {
        arm.setPreset(ArmConstants.PRESETS.MID_SCORE);
      } else if (setHighScore.getAsBoolean()) {
        arm.setPreset(ArmConstants.PRESETS.HIGH_SCORE);
      } else if (wasVelocityControlled) {
        arm.setPos(arm.getPos());
        wasVelocityControlled = false;
      }
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
