// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ArmConstants.GripState;
import frc.robot.subsystems.Arm;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ControlArm extends CommandBase {
  private Arm arm;
  private DoubleSupplier manualControl;
  private BooleanSupplier setStowed, setHandoff, setHighScore, setMedScore, setLowScore;

  private enum Mode {
    POSITION,
    VELOCITY
  }

  private Mode currentMode = Mode.VELOCITY;
  
  public ControlArm(
      DoubleSupplier manualControl, 
      BooleanSupplier setStowed, 
      BooleanSupplier setHandoff, 
      BooleanSupplier setHighScore, 
      BooleanSupplier setMedScore,
      BooleanSupplier setLowScore, 
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

  private void monitorMode() {
    if (setStowed.getAsBoolean() 
        || setHandoff.getAsBoolean() 
        || setHighScore.getAsBoolean() 
        || setMedScore.getAsBoolean() 
        || setLowScore.getAsBoolean()) {
          currentMode = Mode.POSITION;
    } 
    else if (manualControl.getAsDouble() > 0) {
      currentMode = Mode.VELOCITY;
    }
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    monitorMode();

    if(currentMode == Mode.POSITION) {
      if (setStowed.getAsBoolean()) {
        arm.setGrip(GripState.GRIP_OFF);
        arm.setPreset(ArmConstants.PRESETS.STOWED);
      } 
      else if (setHandoff.getAsBoolean()) {
        arm.setGrip(GripState.GRIP_OFF);
        arm.setPreset(ArmConstants.PRESETS.HANDOFF);
      } 
      else if (setHighScore.getAsBoolean()) {
        arm.setGrip(GripState.GRIP_ON);
        arm.setPreset(ArmConstants.PRESETS.HIGH_SCORE);
      } 
      else if (setMedScore.getAsBoolean()) {
        arm.setGrip(GripState.GRIP_ON);
        arm.setPreset(ArmConstants.PRESETS.MID_SCORE);
      } 
      else if (setLowScore.getAsBoolean()) {
        arm.setGrip(GripState.GRIP_ON);
        arm.setPreset(ArmConstants.PRESETS.LOW_SCORE);
      }  else {
        arm.setGrip(GripState.GRIP_OFF);
        arm.setPreset(ArmConstants.PRESETS.STOWED);
      }
    }
    else {
      arm.setSpeed(manualControl.getAsDouble());
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}