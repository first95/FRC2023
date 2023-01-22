// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.LimeLight;
import frc.robot.subsystems.SwerveBase;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SearchForTargets extends CommandBase {
  private final LimeLight _limelight;
  private final SwerveBase _swervebase;

  public SearchForTargets(LimeLight limeLight, SwerveBase swerveBase) {
    _limelight = limeLight;
    _swervebase = swerveBase;
    addRequirements(_limelight, _swervebase);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose3d botpose = _limelight.getBotPose();
    if(botpose != null){
        double timestamp = Timer.getFPGATimestamp() - (_limelight.getTlData() + 11) / 1000;
        _swervebase.updateOdometry(botpose.toPose2d(), timestamp);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
