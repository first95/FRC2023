// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ArmConstants.PRESETS;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class AutoHandoffCube extends SequentialCommandGroup {

    public AutoHandoffCube(Arm arm, Intake intake) {
        addRequirements(arm, intake);

        // Simple cube handoff...
        addCommands(new InstantCommand(() -> {
            intake.setPreset(IntakeConstants.PRESETS.CUBE_HANDOFF);
            arm.setPreset(PRESETS.HANDOFF);
        }));

        addCommands(new WaitUntilCommand(() -> arm.hasReachedReference(PRESETS.HANDOFF.angle())));     

        addCommands(new InstantCommand(() -> {
            intake.runRollers(-0.6);
            arm.setGrip(false);
        }));
        addCommands(new WaitCommand(0.5));
        addCommands(new InstantCommand(() -> intake.runRollers(0)));
    }

}
