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
import frc.robot.Constants;

public class Arm extends SubsystemBase {

  private CANSparkMax armMotor;
  private RelativeEncoder armEncoder;
  private SparkMaxPIDController armController;
  private Solenoid gripper;
  

  public Arm() {
    armMotor = new CANSparkMax(Constants.ArmConstants.ARM_MOTOR_ID, MotorType.kBrushless);
    armEncoder = armMotor.getEncoder();
    armController = armMotor.getPIDController();
    gripper = new Solenoid(PneumaticsModuleType.REVPH, Constants.ArmConstants.GRIP_PNEUMATICS_ID);
    armMotor.restoreFactoryDefaults();
    armMotor.burnFlash();  
  }

  public void setSpeed(double speed){
    armMotor.set(speed);
  }

  public void setPos(double angle){
    armController.setReference(angle, CANSparkMax.ControlType.kPosition);
  }

  public void setPreset(Preset angle){
    double position = angle.pos();
    setPos(position);
  }
  
  public double getPos(){
    return armEncoder.getPosition();
  }
  
  public boolean getGrip(){
    return gripper.get();
  }

  public void setGrip(boolean g){
    gripper.set(g);
  }

  public void toggleGrip(){
    gripper.toggle();
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
