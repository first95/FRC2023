// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.Auton;
import frc.robot.autoCommands.FollowTrajectory;
import frc.robot.subsystems.SwerveBase;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;

public final class Autos {
  /** Example static factory for an autonomous command. */
  public static CommandBase exampleAuto(SwerveBase swerve) {
    PathPlannerTrajectory example = PathPlanner.loadPath("New Path", 
      new PathConstraints(Auton.MAX_SPEED, Auton.MAX_ACCELERATION));
    
    return Commands.sequence(new FollowTrajectory(swerve, example, true));
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }

  
}
