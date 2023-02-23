package frc.robot.autoCommands;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.SwerveBase;

public class Balance extends SequentialCommandGroup {

  public Balance(Alliance alliance, SwerveBase drive) {
    addCommands(new DriveToPose("StartNearBalance", alliance, drive));
    addCommands(new PrecisionAlign("ChargerCenter", alliance, drive));
  }
}