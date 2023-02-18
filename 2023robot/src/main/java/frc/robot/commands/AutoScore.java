// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.RobotContainer;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.Auton;
import frc.robot.Constants.Drivebase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ArmConstants.GripState;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.SwerveBase;

import java.util.Arrays;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class AutoScore extends SequentialCommandGroup {

    public AutoScore(String pose, SwerveBase drive, Arm arm, Intake intake) {
        addRequirements(drive, arm, intake);

        Alliance alliance = DriverStation.getAlliance();
        Pose2d target = Auton.POSE_MAP.get(DriverStation.getAlliance()).get(pose);
        String scorePosition = "high";
        if(pose.contains("Mid"))
            scorePosition = "mid";
        else if(pose.contains("Low"))
            scorePosition = "Low";

        addCommands(new PrecisionAlign(pose, DriverStation.getAlliance(), drive));
        addCommands(new AutoHandoffCone(arm, intake));

        switch (scorePosition) {
            case "high":
                addCommands(new InstantCommand(() -> { arm.setPreset(ArmConstants.PRESETS.HIGH_SCORE); }));
                addCommands(new WaitUntilCommand(arm.hasReachedReference(ArmConstants.PRESETS.HIGH_SCORE.angle())));
                addCommands(new InstantCommand(() -> { arm.setHoldAngle(ArmConstants.PRESETS.HIGH_SCORE.angle()); }));      
                break;
            case "mid":
                addCommands(new InstantCommand(() -> { arm.setPreset(ArmConstants.PRESETS.MID_SCORE); }));
                addCommands(new WaitUntilCommand(arm.hasReachedReference(ArmConstants.PRESETS.MID_SCORE.angle())));
                addCommands(new InstantCommand(() -> { arm.setHoldAngle(ArmConstants.PRESETS.MID_SCORE.angle()); }));     
                break;
            case "low":
                addCommands(new InstantCommand(() -> { arm.setPreset(ArmConstants.PRESETS.LOW_SCORE); }));
                addCommands(new WaitUntilCommand(arm.hasReachedReference(ArmConstants.PRESETS.LOW_SCORE.angle())));
                addCommands(new InstantCommand(() -> { arm.setHoldAngle(ArmConstants.PRESETS.LOW_SCORE.angle()); }));     
                break;
        }

        addCommands(new InstantCommand(() -> {arm.setGrip(GripState.GRIP_OFF);}));
    }

}
