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
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ArmConstants.PRESETS;

public class Arm extends SubsystemBase {
  private double goal = ArmConstants.PRESETS.STOWED.angle();
  private double dt, lastTime, armStowTime, positionSetpointChangedTime;
  private boolean waitingForArmReturn = false;

  private CANSparkMax armMotor;
  private CANSparkMax armMotorFollow;
  private RelativeEncoder armEncoder;
  private SparkMaxPIDController armController;
  private TrapezoidProfile.Constraints armProfileConstraints;
  private TrapezoidProfile armProfile;
  private ArmFeedforward feedforward;
  private DigitalInput cubeSensor;

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
    armProfileConstraints = new TrapezoidProfile.Constraints(
      ArmConstants.ARM_SPEED_LIMIT_RAD_PER_S,
      ArmConstants.ARM_ACCEL_LIMIT_RAD_PER_S);
    
    armController.setP(ArmConstants.ARM_KP);
    armController.setI(ArmConstants.ARM_KI);
    armController.setD(ArmConstants.ARM_KD);
    armController.setFF(ArmConstants.ARM_KF);
    armController.setOutputRange(-ArmConstants.MAX_CONTROL_EFFORT, ArmConstants.MAX_CONTROL_EFFORT);

    armMotor.setSmartCurrentLimit(40);
    armMotor.setIdleMode(IdleMode.kCoast);

    bottomLimitSwitch = armMotor.getReverseLimitSwitch(Type.kNormallyOpen);

    armMotor.burnFlash();

    feedforward = new ArmFeedforward(
      ArmConstants.ARM_KS,
      ArmConstants.ARM_KG,
      ArmConstants.ARM_KV);

    armProfile = new TrapezoidProfile(
      armProfileConstraints,
      new State(ArmConstants.ARM_LOWER_LIMIT, 0),
      new State(armEncoder.getPosition(), armEncoder.getVelocity()));

    cubeSensor = new DigitalInput(ArmConstants.CUBE_SENSOR_ID);
    
    time.reset();
    time.start();
  }

  public boolean getCubeSensor() {
    return cubeSensor.get();
  }

  public double getPos(){
    return armEncoder.getPosition();
  }

  public void setPos(double angleDegree) {
    goal = Math.min(angleDegree, ArmConstants.ARM_UPPER_LIMIT);
    armProfile = new TrapezoidProfile(
      armProfileConstraints,
      new State(goal, 0),
      new State(armEncoder.getPosition(),
      armEncoder.getVelocity()));
    positionSetpointChangedTime = time.get();
  }

  /**
   * Sets velocity by moving position setpoint. Be careful with this- make sure to prevent
   * the setpoint being changed overmuch due to mechanical obstruction.
   * @param velocityRadPerSecond
   */
  public void setVelocity(double velocityRadPerSecond) {
    goal += velocityRadPerSecond * dt;
    if (goal > ArmConstants.ARM_UPPER_LIMIT) {
      goal = ArmConstants.ARM_UPPER_LIMIT;
      velocityRadPerSecond = 0;
    }
    armController.setReference(
      goal,
      ControlType.kPosition,
      0,
      feedforward.calculate(goal, velocityRadPerSecond));
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

  public void setArmVoltage(double volts) {
    armMotor.setVoltage(volts);
  }

  @Override
  public void periodic() {
    if (bottomLimitSwitch.isPressed()) armEncoder.setPosition(PRESETS.STOWED.angle());

    var setpointState = armProfile.calculate(time.get() - positionSetpointChangedTime);
    armController.setReference(
      setpointState.position,
      CANSparkMax.ControlType.kPosition,
      0,
      feedforward.calculate(setpointState.position, setpointState.velocity));

    // Logging...
    SmartDashboard.putBoolean("Gripper Status", gripper.get());
    SmartDashboard.putBoolean("Bottom Limit Switch: ", bottomLimitSwitch.isPressed());
    SmartDashboard.putNumber("Arm Motor Encoder: ", getPos());
    SmartDashboard.putNumber("Arm Current", armMotor.getOutputCurrent());
    SmartDashboard.putNumber("Commanded Arm Voltage", armMotor.getAppliedOutput() * armMotor.getBusVoltage());
    SmartDashboard.putNumber("Arm Setpoint", goal);

    // Report arm position
    Rotation2d currentPosition = Rotation2d.fromDegrees(getPos());
    SmartDashboard.putNumber("armHeight", 
      ArmConstants.SHOULDER_LOCATION.getZ()
      + currentPosition.getSin() * ArmConstants.ARM_LENGTH);
    
    SmartDashboard.putNumber("armExtension",
      ArmConstants.SHOULDER_LOCATION.getX()
      + currentPosition.getCos() * ArmConstants.ARM_LENGTH);

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
