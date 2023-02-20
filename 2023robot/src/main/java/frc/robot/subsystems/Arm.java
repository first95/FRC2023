// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxLimitSwitch.Type;
import com.revrobotics.SparkMaxPIDController.AccelStrategy;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ArmConstants.CONTROL_MODE;
import frc.robot.Constants.ArmConstants.GripState;
import frc.robot.Constants.ArmConstants.PRESETS;

public class Arm extends SubsystemBase {
  private GripState currentGrip = GripState.GRIP_ON;
  private PRESETS currentPosition;
  private double holdAngle = 0;

  private CANSparkMax armMotor;
  private CANSparkMax armMotorFollow;
  private RelativeEncoder armEncoder;
  private SparkMaxPIDController armController;

  private Solenoid gripper;
  private SparkMaxLimitSwitch bottomLimitSwitch;

  public Arm() {
    gripper = new Solenoid(Constants.PNEUMATIC_HUB_ID, PneumaticsModuleType.REVPH, ArmConstants.GRIPPER_SOLENOID_ID);
    armMotor = new CANSparkMax(ArmConstants.ARM_MOTOR_ID, MotorType.kBrushless);
    armMotorFollow = new CANSparkMax(ArmConstants.ARM_MOTOR_FOLLOWER_ID, MotorType.kBrushless);
    armMotorFollow.follow(armMotor, true);
    armMotor.restoreFactoryDefaults();

    armEncoder = armMotor.getEncoder();
    armEncoder.setPositionConversionFactor(ArmConstants.ARM_DEGREES_PER_MOTOR_ROTATION);

    armController = armMotor.getPIDController();
    armController.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, 0);

    armMotor.setSoftLimit(SoftLimitDirection.kForward, ArmConstants.ARM_UPPER_LIMIT);
    armMotor.setSoftLimit(SoftLimitDirection.kReverse, ArmConstants.ARM_LOWER_LIMIT);
    
    applyPID(ArmConstants.ARM_KP, ArmConstants.ARM_KI, ArmConstants.ARM_KD);
    armController.setFF(ArmConstants.ARM_KF);
    armController.setOutputRange(-0.5, 0.5);

    armMotor.setSmartCurrentLimit(30);
    armMotor.setIdleMode(IdleMode.kCoast);

    bottomLimitSwitch = armMotor.getReverseLimitSwitch(Type.kNormallyOpen);

    armMotor.burnFlash();  
  }

  // Allow for setting special PID for certain operations
  // Example: When setting arm to stowed position set P value lower
  //          to prevent slamming into back.
  public void applyPID(double p, double I, double D) {
    armController.setP(p);
    armController.setI(I);
    armController.setD(D);
    armMotor.burnFlash();  
  }

  public void setBreaks(boolean enabled) {
    armMotor.setIdleMode(enabled ? IdleMode.kBrake : IdleMode.kCoast);
  }

  public void setSpeed(double speed){
    armMotor.set(speed);
  }

  public void setPreset(ArmConstants.PRESETS position){
    setHoldAngle(position.angle());
    setPos(position.angle());
    currentPosition = position;
  }
  
  public BooleanSupplier hasReachedReference(double reference) {
    return () -> { return armMotor.getEncoder().getPosition() + 3 > (reference)
      && armMotor.getEncoder().getPosition() -3 < (reference); };  
    }

  public double getPos(){
    return armEncoder.getPosition();
  }

  public void setPos(double angleDegree){
    armController.setReference(angleDegree, CANSparkMax.ControlType.kPosition);
  }

  public double getHoldAngle() {
    return holdAngle;
  }

  public void setHoldAngle(double newHoldAngle) {
    holdAngle = newHoldAngle;
  }
  
  public GripState getGrip(){
    return currentGrip;
  }

  public void setGrip(GripState state){
    if(currentGrip == GripState.GRIP_ON)
      gripper.set(true);
    else if(currentGrip == GripState.GRIP_OFF)
      gripper.set(false);
    currentGrip = state;
  }

  public void toggleGrip() {
    if(currentGrip == GripState.GRIP_OFF)
      setGrip(GripState.GRIP_ON);
    else
      setGrip(GripState.GRIP_OFF);
  }

  @Override
  public void periodic() {
    if (bottomLimitSwitch.isPressed()) armEncoder.setPosition(0);

    // Logging...
    SmartDashboard.putBoolean("Bottom Limit Switch: ", bottomLimitSwitch.isPressed());
    SmartDashboard.putNumber("Arm Motor Encoder: ", getPos());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}

// See intake retracted, then move arm at the same time (For cones only)
