// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class AutoHandoffCube extends SequentialCommandGroup {

    public AutoHandoffCube(Arm arm, Intake intake) {
        addRequirements(arm, intake);

        // Simple cube handoff...
        addCommands(new InstantCommand(() -> {
            intake.setPreset(IntakeConstants.PRESETS.CUBE);
            intake.grabCube(0.6);
        }));

        addCommands(new WaitCommand(0.5));
        addCommands(new InstantCommand(() -> {arm.setPos(-45);}));
        addCommands(new WaitUntilCommand(() -> arm.hasReachedReference(-55)));     

        addCommands(new InstantCommand(() -> {
            intake.grabCube(0);
            intake.setPreset(IntakeConstants.PRESETS.STOWED);
        }));
        addCommands(new InstantCommand(() -> SmartDashboard.putString("LastHandoff", "CUBE")));

    }

}
