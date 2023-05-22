// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class AutoHandoffCube extends SequentialCommandGroup {

    public AutoHandoffCube(Arm arm, Intake intake) {
        addRequirements(arm, intake);

        // Simple cube handoff...
        addCommands(new InstantCommand(() -> {
            intake.setPreset(IntakeConstants.PRESETS.CUBE);
            intake.runRollers(0.6);
        }));

        addCommands(new InstantCommand(() -> {arm.setPos(-55);}));
        addCommands(new WaitUntilCommand(() -> arm.hasReachedReference(-60)));     

        addCommands(new InstantCommand(() -> {
            intake.runRollers(0);
            intake.setPreset(IntakeConstants.PRESETS.STOWED);
        }));

    }

}
