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
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ArmConstants.GripState;

public class Arm extends SubsystemBase {

  private GripState currentGrip = GripState.GRIP_OFF;
  private CANSparkMax armMotor;
  private CANSparkMax armMotorFollow;
  private RelativeEncoder armEncoder;
  private SparkMaxPIDController armController;
  private Solenoid gripper;
  private DigitalInput toplimitSwitch = new DigitalInput(0);
  private DigitalInput bottomlimitSwitch = new DigitalInput(1);
  private ArmFeedforward feedForward = new ArmFeedforward(ArmConstants.ARM_KS, ArmConstants.ARM_KG, ArmConstants.ARM_KV);
  

  public Arm() {
    armMotor = new CANSparkMax(ArmConstants.ARM_MOTOR_ID, MotorType.kBrushless);
    armMotorFollow = new CANSparkMax(ArmConstants.ARM_MOTOR_FOLLOW_ID, MotorType.kBrushless);
    armMotorFollow.follow(armMotor, true);
    armEncoder = armMotor.getEncoder();
    armController = armMotor.getPIDController();
    gripper = new Solenoid(60, PneumaticsModuleType.REVPH, ArmConstants.GRIPPER_SOLENOID_ID);
    armMotor.restoreFactoryDefaults();
    armController.setP(.001);
    armController.setI(0);
    armController.setD(0);
    armMotor.burnFlash();  
  }

  public void setBreaks(boolean enabled) {
    armMotor.setIdleMode(enabled ? IdleMode.kBrake : IdleMode.kCoast);
    armMotorFollow.setIdleMode(enabled ? IdleMode.kBrake : IdleMode.kCoast);
  }

  public void setSpeed(double speed){
    armMotor.set(speed);
  }

  public void setVoltage(double setpoint){
    armMotor.setVoltage(feedForward.calculate(setpoint, ArmConstants.ARM_KV));
  }

  public void setPreset(ArmConstants.Preset position){
    double angle = position.angle();
    setPos(angle);
  }
  
  public double getPos(){
    return armEncoder.getPosition();
  }

  public void setPos(double rotation){
    armController.setReference(rotation, CANSparkMax.ControlType.kPosition);
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
