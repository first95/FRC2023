// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ArmConstants.GripState;

public class Arm extends SubsystemBase {

  private GripState currentGrip = GripState.GRIP_OFF;
  private CANSparkMax armMotor;
  private CANSparkMax armMotorFollow;
  private RelativeEncoder armEncoder;
  private SparkMaxPIDController armController;
  private Solenoid coneGripper;
  private Solenoid cubeGripper;
  private DigitalInput toplimitSwitch = new DigitalInput(0);
  private DigitalInput bottomlimitSwitch = new DigitalInput(1);
  private ArmFeedforward feedForward = new ArmFeedforward(ArmConstants.ARM_KS, ArmConstants.ARM_KG, ArmConstants.ARM_KV);
  

  public Arm() {
    armMotor = new CANSparkMax(ArmConstants.ARM_MOTOR_ID, MotorType.kBrushless);
    armMotorFollow = new CANSparkMax(ArmConstants.ARM_MOTOR_FOLLOW_ID, MotorType.kBrushless);
    armMotorFollow.follow(armMotor, true);
    armEncoder = armMotor.getEncoder();
    armController = armMotor.getPIDController();
    coneGripper = new Solenoid(PneumaticsModuleType.REVPH, ArmConstants.CUBE_SOLENOID_ID);
    cubeGripper = new Solenoid(PneumaticsModuleType.REVPH, ArmConstants.CONE_SOLENOID_ID);
    armMotor.restoreFactoryDefaults();
    armController.setP(ArmConstants.ARM_KP);
    armController.setI(ArmConstants.ARM_KI);
    armController.setD(ArmConstants.ARM_KD);
    armMotor.burnFlash();  
  }

  public void setSpeed(double speed){
    if (speed > 0) {
      if (toplimitSwitch.get()) {
          // We are going up and top limit is tripped so stop
          armMotor.set(0);
      } else {
          // We are going up but top limit is not tripped so go at commanded speed
          armMotor.set(speed);
      }
  } else {
      if (bottomlimitSwitch.get()) {
          // We are going down and bottom limit is tripped so stop
          armMotor.set(0);
      } else {
          // We are going down but bottom limit is not tripped so go at commanded speed
          armMotor.set(speed);
      }
  }
  }

  public void setVoltage(double p, double v){
    armMotor.setVoltage(feedForward.calculate(p,v));
  }


  public void setPos(double angle){
    armController.setReference(angle, CANSparkMax.ControlType.kPosition);
  }

  public void setPreset(ArmConstants.Preset position){
    double angle = position.angle();
    setPos(angle);
  }
  
  public double getPos(){
    return armEncoder.getPosition();
  }
  
  public GripState getGrip(){
    return currentGrip;
  }

  public void setGrip(GripState state){
    if(currentGrip == GripState.GRIP_CUBE ){
      coneGripper.set(false);
      cubeGripper.set(true);
    }else if(currentGrip == GripState.GRIP_CONE){
      coneGripper.set(true);
      cubeGripper.set(false);
    }else{
      coneGripper.set(false);
      cubeGripper.set(false);
    }
    currentGrip = state;
  }

  public void resetOdometry() {
    armEncoder.setPosition(0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Arm Pos", getPos());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}