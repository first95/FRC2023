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
  private double holdAngle = 0;
  private PRESETS currentPreset;

  private CANSparkMax armMotor;
  private CANSparkMax armMotorFollow;
  private RelativeEncoder armEncoder;
  private SparkMaxPIDController armController;

  private Solenoid gripper; // FALSE is CLOSE || TRUE is OPEN
  private SparkMaxLimitSwitch bottomLimitSwitch;

  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, maxVel, minVel, maxAcc, allowedErr;

  public Arm() {
    gripper = new Solenoid(Constants.PNEUMATIC_HUB_ID, PneumaticsModuleType.REVPH, ArmConstants.GRIPPER_SOLENOID_ID);
    armMotor = new CANSparkMax(ArmConstants.ARM_MOTOR_ID, MotorType.kBrushless);
    armMotorFollow = new CANSparkMax(ArmConstants.ARM_MOTOR_FOLLOWER_ID, MotorType.kBrushless);

    armMotorFollow.follow(armMotor, true);
    armMotor.restoreFactoryDefaults();
    armMotor.setSoftLimit(SoftLimitDirection.kForward, ArmConstants.ARM_UPPER_LIMIT);
    armMotor.setSoftLimit(SoftLimitDirection.kReverse, ArmConstants.ARM_LOWER_LIMIT);

    armEncoder = armMotor.getEncoder();
    armEncoder.setPositionConversionFactor(ArmConstants.ARM_DEGREES_PER_MOTOR_ROTATION);
    armEncoder.setPositionConversionFactor(ArmConstants.ARM_DEGREES_PER_MOTOR_ROTATION / 60);


    armController = armMotor.getPIDController();
    
    // TESTING SMART MOTION //
    kP = 5e-5; 
    kI = 1e-6;
    kD = 0; 
    kIz = 0; 
    kFF = 0.000156; 
    kMaxOutput = 0.05; 
    kMinOutput = -0.05;
    maxRPM = 5700;

    // Smart Motion Coefficients
    maxVel = 60;    // Degrees per minute
    maxAcc = 45;    // Degrees per minute per second
    allowedErr = 2; // Degrees

    armController.setP(kP);
    armController.setI(kI);
    armController.setD(kD);
    armController.setIZone(kIz);
    armController.setFF(kFF);
    armController.setOutputRange(kMinOutput, kMaxOutput);

    armController.setSmartMotionAccelStrategy(AccelStrategy.kSCurve, 0);
    armController.setSmartMotionMaxVelocity(maxVel, 0);
    armController.setSmartMotionMinOutputVelocity(minVel, 0);
    armController.setSmartMotionMaxAccel(maxAcc, 0);
    armController.setSmartMotionAllowedClosedLoopError(allowedErr, 0);

    // display PID coefficients on SmartDashboard
    SmartDashboard.putNumber("ARM P Gain", kP);
    SmartDashboard.putNumber("ARM I Gain", kI);
    SmartDashboard.putNumber("ARM D Gain", kD);
    SmartDashboard.putNumber("ARM I Zone", kIz);
    SmartDashboard.putNumber("ARM Feed Forward", kFF);
    SmartDashboard.putNumber("ARM Max Output", kMaxOutput);
    SmartDashboard.putNumber("ARM Min Output", kMinOutput);

    // display Smart Motion coefficients
    SmartDashboard.putNumber("ARM Max Velocity", maxVel);
    SmartDashboard.putNumber("ARM Min Velocity", minVel);
    SmartDashboard.putNumber("ARM Max Acceleration", maxAcc);
    SmartDashboard.putNumber("ARM Allowed Closed Loop Error", allowedErr);
    SmartDashboard.putNumber("ARM Set Position", 0);
    SmartDashboard.putNumber("ARM Set Velocity", 0);
    /////////////////////////

    // applyPID(ArmConstants.ARM_KP, ArmConstants.ARM_KI, ArmConstants.ARM_KD);
    // armController.setOutputRange(-0.2, 0.2);

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

  public void setVelocity(double velocity){
    SmartDashboard.putNumber("Commanded Velocity: ", velocity);
    applyPID(0.01, ArmConstants.ARM_KI, 0.001);
    armController.setReference(velocity, CANSparkMax.ControlType.kVelocity);
  }

  public void setPreset(ArmConstants.PRESETS position){
    setHoldAngle(position.angle());
    setPos(position.angle());
    currentPreset = position;
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

  @Override
  public void periodic() {
    if (bottomLimitSwitch.isPressed()) armEncoder.setPosition(0);

    // read PID coefficients from SmartDashboard
    double p = SmartDashboard.getNumber("ARM P Gain", 0);
    double i = SmartDashboard.getNumber("ARM I Gain", 0);
    double d = SmartDashboard.getNumber("ARM D Gain", 0);
    double iz = SmartDashboard.getNumber("ARM I Zone", 0);
    double ff = SmartDashboard.getNumber("ARM Feed Forward", 0);
    double max = SmartDashboard.getNumber("ARM Max Output", 0);
    double min = SmartDashboard.getNumber("ARM Min Output", 0);
    double maxV = SmartDashboard.getNumber("ARM Max Velocity", 0);
    double minV = SmartDashboard.getNumber("ARM Min Velocity", 0);
    double maxA = SmartDashboard.getNumber("ARM Max Acceleration", 0);
    double allE = SmartDashboard.getNumber("ARM Allowed Closed Loop Error", 0);

    // if PID coefficients on SmartDashboard have changed, write new values to controller
    if((p != kP)) { armController.setP(p); kP = p; }
    if((i != kI)) { armController.setI(i); kI = i; }
    if((d != kD)) { armController.setD(d); kD = d; }
    if((iz != kIz)) { armController.setIZone(iz); kIz = iz; }
    if((ff != kFF)) { armController.setFF(ff); kFF = ff; }
    if((max != kMaxOutput) || (min != kMinOutput)) { armController.setOutputRange(min, max); kMinOutput = min; kMaxOutput = max; }

    if((maxV != maxVel)) { armController.setSmartMotionMaxVelocity(maxV,0); maxVel = maxV; }
    if((minV != minVel)) { armController.setSmartMotionMinOutputVelocity(minV,0); minVel = minV; }
    if((maxA != maxAcc)) { armController.setSmartMotionMaxAccel(maxA,0); maxAcc = maxA; }
    if((allE != allowedErr)) { armController.setSmartMotionAllowedClosedLoopError(allE,0); allowedErr = allE; }

    // Logging...
    SmartDashboard.putBoolean("Gripper Status", gripper.get());
    SmartDashboard.putBoolean("Bottom Limit Switch: ", bottomLimitSwitch.isPressed());
    SmartDashboard.putNumber("Arm Motor Encoder: ", getPos());
    SmartDashboard.putNumber("Arm Motor Encoder Velocity: ", armEncoder.getVelocity());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}