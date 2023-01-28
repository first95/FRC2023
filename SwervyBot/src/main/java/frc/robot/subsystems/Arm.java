// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ArmConstants.GripState;

public class Arm extends SubsystemBase {

  private GripState currentGrip;
  private CANSparkMax armMotor;
  private RelativeEncoder armEncoder;
  private SparkMaxPIDController armController;
  private Solenoid coneGripper;
  private Solenoid cubeGripper;
  

  public Arm() {
    armMotor = new CANSparkMax(ArmConstants.ARM_MOTOR_ID, MotorType.kBrushless);
    armEncoder = armMotor.getEncoder();
    armController = armMotor.getPIDController();
    coneGripper = new Solenoid(PneumaticsModuleType.REVPH, ArmConstants.GRIP_PNEUMATICS_ID);
    cubeGripper = new Solenoid(PneumaticsModuleType.REVPH, ArmConstants.GRIP_PNEUMATICS_ID);
    armMotor.restoreFactoryDefaults();
    armController.setP(ArmConstants.ARM_KP);
    armController.setP(ArmConstants.ARM_KI);
    armController.setP(ArmConstants.ARM_KD);
    armMotor.burnFlash();  
  }

  public void setSpeed(double speed){
    armMotor.set(speed);
  }

  public void setPos(double angle){
    armController.setReference(angle, CANSparkMax.ControlType.kPosition);
  }

  public void setGrip(ArmConstants.Preset position){
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
    currentGrip = state;
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
