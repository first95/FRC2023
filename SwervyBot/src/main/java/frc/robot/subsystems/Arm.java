// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;



public class Arm extends SubsystemBase {

  private CANSparkMax motor;
  private RelativeEncoder encoder;

  public Arm() {
    motor = new CANSparkMax(Constants.ARM_MOTOR_ID, MotorType.kBrushless);
    encoder = motor.getEncoder();
  }

  public void setSpeed(double speed){
    motor.set(speed);
  }

  public void setPos(double position){
    encoder.setPosition(position);
  }

  public double getPos(){
    return encoder.getPosition();
  }

  public void resetOdometry() {
    encoder.setPosition(0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Arm Pos", -encoder.getPosition());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
