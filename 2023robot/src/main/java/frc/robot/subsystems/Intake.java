// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;;

public class Intake extends SubsystemBase {
  private CANSparkMax bottomRoller, topRoller;
  /** Creates a new Intake subsystem. */
  public Intake() {
    bottomRoller = new CANSparkMax(IntakeConstants.BOTTOM_ROLLER_ID, MotorType.kBrushless);
    topRoller = new CANSparkMax(IntakeConstants.TOP_ROLLER_ID, MotorType.kBrushless);

    bottomRoller.restoreFactoryDefaults();
    bottomRoller.setIdleMode(IdleMode.kBrake);
    bottomRoller.setInverted(IntakeConstants.INVERT_ROLLERS);
    bottomRoller.burnFlash();

    topRoller.restoreFactoryDefaults();
    topRoller.setIdleMode(IdleMode.kBrake);
    topRoller.setInverted(!IntakeConstants.INVERT_ROLLERS);
    topRoller.burnFlash();
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
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
