// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxLimitSwitch.Type;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.IntakeConstants.PRESETS;

public class Intake extends SubsystemBase {
  private CANSparkMax rollers, rotator, gearRack;
  private SparkMaxPIDController rackController, rotatorController;
  private RelativeEncoder rackEncoder, rotatorEncoder;
  private SparkMaxLimitSwitch homeSwitch;
  /** Creates a new Intake subsystem. */
  public Intake() {
    rollers = new CANSparkMax(IntakeConstants.ROLLER_ID, MotorType.kBrushless);
    rotator = new CANSparkMax(IntakeConstants.ROTATOR_ID, MotorType.kBrushless);
    gearRack = new CANSparkMax(IntakeConstants.RACK_ID, MotorType.kBrushless);

    rollers.restoreFactoryDefaults();
    rollers.setIdleMode(IdleMode.kBrake);
    rollers.setInverted(IntakeConstants.INVERT_ROLLERS);
    rollers.setSmartCurrentLimit(15);
    rollers.burnFlash();

    rotator.restoreFactoryDefaults();
    rotator.setIdleMode(IdleMode.kBrake);
    rotator.setInverted(IntakeConstants.INVERT_ROTATOR);
    rotator.setSmartCurrentLimit(15);

    gearRack.restoreFactoryDefaults();
    gearRack.setIdleMode(IdleMode.kCoast);
    gearRack.setInverted(IntakeConstants.INVERT_RACK);
    gearRack.setSmartCurrentLimit(20);

    rackEncoder = gearRack.getEncoder();
    rackEncoder.setPositionConversionFactor(IntakeConstants.RACK_METERS_PER_MOTOR_ROTATION);
    rackEncoder.setVelocityConversionFactor(IntakeConstants.RACK_METERS_PER_MOTOR_ROTATION / 60);
    rackEncoder.setPosition(0);

    rotatorEncoder = rotator.getEncoder();
    rotatorEncoder.setPositionConversionFactor(IntakeConstants.ROTATOR_DEGREES_PER_MOTOR_ROTATION);
    rotatorEncoder.setVelocityConversionFactor(IntakeConstants.ROTATOR_DEGREES_PER_MOTOR_ROTATION / 60);
    rotatorEncoder.setPosition(0);

    gearRack.setSoftLimit(SoftLimitDirection.kForward, IntakeConstants.RACK_UPPER_LIMIT);
    gearRack.setSoftLimit(SoftLimitDirection.kReverse, IntakeConstants.RACK_LOWER_LIMIT);
    
    rackController = gearRack.getPIDController();
    rackController.setP(IntakeConstants.KP);
    rackController.setI(IntakeConstants.KI);
    rackController.setD(IntakeConstants.KD);
    rackController.setFF(IntakeConstants.KF);
    rackController.setIZone(IntakeConstants.IZ);

    homeSwitch = gearRack.getReverseLimitSwitch(Type.kNormallyOpen);

    rotatorController = rotator.getPIDController();
    rotatorController.setP(IntakeConstants.ROTATOR_KP);
    rotatorController.setI(IntakeConstants.ROTATOR_KI);
    rotatorController.setD(IntakeConstants.ROTATOR_KD);
    rotatorController.setFF(IntakeConstants.ROTATOR_KF);
    rotatorController.setIZone(IntakeConstants.ROTATOR_IZ);
    /*rotatorController.setPositionPIDWrappingEnabled(true);
    rotatorController.setPositionPIDWrappingMinInput(-180);
    rotatorController.setPositionPIDWrappingMaxInput(180);*/

    gearRack.burnFlash();
  }

  public void runRollers(double speed) {
    rollers.set(speed);
  }

  public void setPosition(double meters) {
    rackController.setReference(meters, ControlType.kPosition);
  }

  public void setRotator(double degrees) {
    rotatorController.setReference(degrees, ControlType.kPosition);
  }

  public void setPreset(PRESETS preset) {
    setPosition(preset.position());
    setRotator(preset.angle());
  }

  public double getRollerCurrentDraw() {
    return rollers.getOutputCurrent();
  }

  public boolean rackHasReachedReference(double reference) {
    return rackEncoder.getPosition() + 0.025 > (reference)
      && rackEncoder.getPosition() - 0.025 < (reference);
  }

  public void moveIntake(double speed) {
    gearRack.set(speed * 0.3);
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public CommandBase exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (homeSwitch.isPressed()) {
      rackEncoder.setPosition(0);
    }

    // Logging...
    SmartDashboard.putNumber("Rack Encoder", rackEncoder.getPosition());
    SmartDashboard.putNumber("Roller Current Draw", getRollerCurrentDraw());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
