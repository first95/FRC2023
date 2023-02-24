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
  private CANSparkMax bottomRoller, topRoller, gearRack;
  private SparkMaxPIDController rackController;
  private RelativeEncoder rackEncoder;
  private SparkMaxLimitSwitch homeSwitch;
  /** Creates a new Intake subsystem. */
  public Intake() {
    bottomRoller = new CANSparkMax(IntakeConstants.BOTTOM_ROLLER_ID, MotorType.kBrushless);
    topRoller = new CANSparkMax(IntakeConstants.TOP_ROLLER_ID, MotorType.kBrushless);
    gearRack = new CANSparkMax(IntakeConstants.RACK_ID, MotorType.kBrushless);

    bottomRoller.restoreFactoryDefaults();
    bottomRoller.setIdleMode(IdleMode.kBrake);
    bottomRoller.setInverted(IntakeConstants.INVERT_ROLLERS);
    bottomRoller.setSmartCurrentLimit(15);
    bottomRoller.burnFlash();

    topRoller.restoreFactoryDefaults();
    topRoller.setIdleMode(IdleMode.kBrake);
    topRoller.setInverted(!IntakeConstants.INVERT_ROLLERS);
    topRoller.setSmartCurrentLimit(15);
    topRoller.burnFlash();

    gearRack.restoreFactoryDefaults();
    gearRack.setIdleMode(IdleMode.kCoast);
    gearRack.setInverted(IntakeConstants.INVERT_RACK);
    gearRack.setSmartCurrentLimit(20);

    rackEncoder = gearRack.getEncoder();
    rackEncoder.setPositionConversionFactor(IntakeConstants.RACK_METERS_PER_MOTOR_ROTATION);
    rackEncoder.setVelocityConversionFactor(IntakeConstants.RACK_METERS_PER_MOTOR_ROTATION / 60);
    rackEncoder.setPosition(0);

    gearRack.setSoftLimit(SoftLimitDirection.kForward, IntakeConstants.RACK_UPPER_LIMIT);
    gearRack.setSoftLimit(SoftLimitDirection.kReverse, IntakeConstants.RACK_LOWER_LIMIT);
    
    rackController = gearRack.getPIDController();
    rackController.setP(IntakeConstants.KP);
    rackController.setI(IntakeConstants.KI);
    rackController.setD(IntakeConstants.KD);
    rackController.setFF(IntakeConstants.KF);
    rackController.setIZone(IntakeConstants.IZ);

    homeSwitch = gearRack.getReverseLimitSwitch(Type.kNormallyOpen);

    gearRack.burnFlash();
  }

  public void grabCube(double speed) {
    bottomRoller.set(speed);
    topRoller.set(speed);
  }

  public void grabCone(double speed) {
    bottomRoller.set(-speed);
    topRoller.set(speed);
  }

  public void runRollers(double coneSpeed, double cubeSpeed) {
    bottomRoller.set(cubeSpeed - coneSpeed);
    topRoller.set(cubeSpeed + coneSpeed);
  }

  public void setPosition(double meters) {
    rackController.setReference(meters, ControlType.kPosition);
  }

  public void setPreset(PRESETS preset) {
    setPosition(preset.position());
  }

  public BooleanSupplier rackHasReachedReference(double reference) {
    return () -> { return rackEncoder.getPosition() + 0.025 > (reference)
      && rackEncoder.getPosition() - 0.025 < (reference); };  
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
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
