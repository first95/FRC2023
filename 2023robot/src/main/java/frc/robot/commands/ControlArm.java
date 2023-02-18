// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ArmConstants.CONTROL_MODE;
import frc.robot.Constants.ArmConstants.GripState;
import frc.robot.subsystems.Arm;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ControlArm extends CommandBase {
  private Arm arm;
  private DoubleSupplier manualControl;
  private BooleanSupplier setStowed, setHandoff, setHighScore, setMedScore, setLowScore;

  private CONTROL_MODE currentMode = CONTROL_MODE.DUTY;
  private double holdAngle = 0;
  
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
    // Setpoints
    if ((setStowed.getAsBoolean() 
        || setHandoff.getAsBoolean() 
        || setHighScore.getAsBoolean() 
        || setMedScore.getAsBoolean() 
        || setLowScore.getAsBoolean()) 
        && currentMode != CONTROL_MODE.POSITION) {
          currentMode = CONTROL_MODE.POSITION;
    } 
    // Joystick duty override
    else if (Math.abs(manualControl.getAsDouble()) > 0) {
      currentMode = CONTROL_MODE.DUTY;
    }
    else {
      if (currentMode == CONTROL_MODE.DUTY) {
        currentMode = CONTROL_MODE.HOLD;
        holdAngle = arm.getPos();
      }
    }
  }

  @Override
  public void initialize() {
    arm.setPos(0);
  }

  @Override
  public void execute() {
    monitorMode();

    if(currentMode == CONTROL_MODE.POSITION) {
      if (setStowed.getAsBoolean()) {
        arm.setPreset(ArmConstants.PRESETS.STOWED);
      } 
      else if (setHandoff.getAsBoolean()) {
        arm.setPreset(ArmConstants.PRESETS.HANDOFF);
      } 
      else if (setHighScore.getAsBoolean()) {
        arm.setPreset(ArmConstants.PRESETS.HIGH_SCORE);
      } 
      else if (setMedScore.getAsBoolean()) {
        arm.setPreset(ArmConstants.PRESETS.MID_SCORE);
      } 
      else if (setLowScore.getAsBoolean()) {
        arm.setPreset(ArmConstants.PRESETS.LOW_SCORE);
      }
    }
    else if (currentMode == CONTROL_MODE.HOLD)
      arm.setPos(holdAngle);
    else if(currentMode == CONTROL_MODE.DUTY) {
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
