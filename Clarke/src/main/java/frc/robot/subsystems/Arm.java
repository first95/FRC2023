// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxLimitSwitch.Type;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ArmConstants.PRESETS;

public class Arm extends SubsystemBase {
  private double setPoint = ArmConstants.PRESETS.STOWED.angle();
  private double dt, lastTime, armStowTime;
  private boolean waitingForArmReturn = false;

  private CANSparkMax armMotor;
  private CANSparkMax armMotorFollow;
  private RelativeEncoder armEncoder;
  private SparkMaxPIDController armController;
  private ArmFeedforward feedforward;

  private Timer time = new Timer();

  private Solenoid gripper; // FALSE is CLOSE || TRUE is OPEN
  private SparkMaxLimitSwitch bottomLimitSwitch;

  public Arm() {
    gripper = new Solenoid(Constants.PNEUMATIC_HUB_ID, PneumaticsModuleType.REVPH, ArmConstants.GRIPPER_SOLENOID_ID);
    armMotor = new CANSparkMax(ArmConstants.ARM_MOTOR_ID, MotorType.kBrushless);
    armMotorFollow = new CANSparkMax(ArmConstants.ARM_MOTOR_FOLLOWER_ID, MotorType.kBrushless);
    armMotorFollow.follow(armMotor, true);
    armMotor.restoreFactoryDefaults();

    armEncoder = armMotor.getEncoder();
    armEncoder.setPositionConversionFactor(ArmConstants.ARM_DEGREES_PER_MOTOR_ROTATION);
    armEncoder.setVelocityConversionFactor(ArmConstants.ARM_DEGREES_PER_MOTOR_ROTATION / 60);

    armController = armMotor.getPIDController();
    
    armController.setP(ArmConstants.ARM_KP);
    armController.setI(ArmConstants.ARM_KI);
    armController.setD(ArmConstants.ARM_KD);
    armController.setFF(ArmConstants.ARM_KF);
    armController.setOutputRange(-ArmConstants.MAX_CONTROL_EFFORT, ArmConstants.MAX_CONTROL_EFFORT);

    armMotor.setSmartCurrentLimit(30);
    armMotor.setIdleMode(IdleMode.kCoast);

    bottomLimitSwitch = armMotor.getReverseLimitSwitch(Type.kNormallyOpen);

    armMotor.burnFlash();

    feedforward = new ArmFeedforward(
      ArmConstants.ARM_KS,
      ArmConstants.ARM_KG,
      ArmConstants.ARM_KV);
    
    time.reset();
    time.start();
  }

  public double getPos(){
    return armEncoder.getPosition();
  }

  public void setPos(double angleDegree) {
    setPoint = Math.min(angleDegree, ArmConstants.ARM_UPPER_LIMIT);
    armController.setReference(
      angleDegree,
      CANSparkMax.ControlType.kPosition,
      0,
      feedforward.calculate(Math.toRadians(setPoint), 0));
  }

  /**
   * Sets velocity by moving position setpoint. Be careful with this- make sure to prevent
   * the setpoint being changed overmuch due to mechanical obstruction.
   * @param velocityDegPerSecond
   */
  public void setVelocity(double velocityDegPerSecond) {
    setPoint += velocityDegPerSecond * dt;
    if (setPoint > ArmConstants.ARM_UPPER_LIMIT) {
      setPoint = ArmConstants.ARM_UPPER_LIMIT;
      velocityDegPerSecond = 0;
    }
    armController.setReference(
      setPoint,
      ControlType.kPosition,
      0,
      feedforward.calculate(Math.toRadians(setPoint), Math.toRadians(velocityDegPerSecond)));
  }
  
  public void setPreset(PRESETS preset) {
    if (preset == ArmConstants.PRESETS.STOWED && getPos() > ArmConstants.RETURN_MIDPOINT) {
      setPos(ArmConstants.RETURN_MIDPOINT);
      waitingForArmReturn = true;
      armStowTime = time.get();
    } else {
      setPos(preset.angle());
    }
  }

  public void toggleGrip() {
    if(gripper.get())
      gripper.set(false);
    else
      gripper.set(true);
  }

  public boolean getGrip(){
    return gripper.get();
  }

  public void setGrip(Boolean isOpen){
    gripper.set(isOpen);
  }

  public void setBrakes(boolean brake) {
    armMotor.setIdleMode(brake ? IdleMode.kBrake : IdleMode.kCoast);
    armMotorFollow.setIdleMode(brake ? IdleMode.kBrake : IdleMode.kCoast);
  }

  @Override
  public void periodic() {
    if (bottomLimitSwitch.isPressed()) armEncoder.setPosition(PRESETS.STOWED.angle());

    if (waitingForArmReturn && (time.get() - armStowTime >= ArmConstants.RETURN_TIME_STOWING)) {
      setPos(ArmConstants.PRESETS.STOWED.angle());
      waitingForArmReturn = false;
    }

    // Logging...
    SmartDashboard.putBoolean("Gripper Status", gripper.get());
    SmartDashboard.putBoolean("Bottom Limit Switch: ", bottomLimitSwitch.isPressed());
    SmartDashboard.putNumber("Arm Motor Encoder: ", getPos());

    dt = time.get() - lastTime;
    lastTime = time.get();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public boolean hasReachedReference(double angle) {
    return Boolean.valueOf(Math.abs(getPos() - angle) <= ArmConstants.ANGLE_TOLERANCE);
  }
}
