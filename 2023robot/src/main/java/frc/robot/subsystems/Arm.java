// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

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

public class Arm extends SubsystemBase {
  private GripState currentGrip = GripState.GRIP_OFF;

  private CANSparkMax armMotor;
  private CANSparkMax armMotorFollow;
  private RelativeEncoder armEncoder;
  private SparkMaxPIDController armController;

  private Solenoid gripper;

  private DigitalInput bottomLimitSwitch = new DigitalInput(1);

  public Arm() {
    gripper = new Solenoid(Constants.PNEUMATIC_HUB_ID, PneumaticsModuleType.REVPH, ArmConstants.GRIPPER_SOLENOID_ID);
    armMotor = new CANSparkMax(ArmConstants.ARM_MOTOR_ID, MotorType.kBrushless);
    armMotorFollow = new CANSparkMax(ArmConstants.ARM_MOTOR_FOLLOWER_ID, MotorType.kBrushless);
    armMotorFollow.follow(armMotor, true);
    armMotor.restoreFactoryDefaults();

    armEncoder = armMotor.getEncoder();
    armEncoder.setPositionConversionFactor(ArmConstants.ARM_DEGREES_PER_MOTOR_ROTATION);
    armController = armMotor.getPIDController();
    
    armController.setP(ArmConstants.ARM_KP);
    armController.setI(ArmConstants.ARM_KI);
    armController.setD(ArmConstants.ARM_KD);

    armMotor.setSmartCurrentLimit(20);
    armMotor.setIdleMode(IdleMode.kCoast);

    armMotor.burnFlash();  
  }

  public void setControlMode(CONTROL_MODE mode) {
    if(mode == CONTROL_MODE.POSITION)
      armController.setReference(getPos(), CANSparkMax.ControlType.kPosition);
    else if(mode == CONTROL_MODE.DUTY)
      armController.setReference(0, CANSparkMax.ControlType.kDutyCycle);
  }

  public void setBreaks(boolean enabled) {
    armMotor.setIdleMode(enabled ? IdleMode.kBrake : IdleMode.kCoast);
  }

  public void setSpeed(double speed){
    armMotor.set(speed);
  }

  public void setPreset(ArmConstants.PRESETS position){
    setPos(position.angle());
  }
  
  public double getPos(){
    return armEncoder.getPosition();
  }

  public void setPos(double angleDegree){
    armController.setReference(angleDegree, CANSparkMax.ControlType.kPosition);
  }
  
  public GripState getGrip(){
    return currentGrip;
  }

  public void setGrip(GripState state){
    System.out.println("Gripper State: " + state);
    if(currentGrip == GripState.GRIP_ON)
      gripper.set(true);
    else if(currentGrip == GripState.GRIP_OFF)
      gripper.set(false);
    currentGrip = state;
  }

  public void toggleGrip() {
    System.out.println("Toggling Grip");
    if(currentGrip == GripState.GRIP_OFF)
      setGrip(GripState.GRIP_ON);
    else
      setGrip(GripState.GRIP_OFF);
  }

  @Override
  public void periodic() {
    if (bottomLimitSwitch.get()) armEncoder.setPosition(0);

    // Logging...
    SmartDashboard.putBoolean("Bottom Limit Switch:: ", bottomLimitSwitch.get());
    SmartDashboard.putNumber("Arm Motor Encoder: ", getPos());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
