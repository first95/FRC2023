// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.RobotContainer;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;

import java.util.Arrays;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class AutoEject extends SequentialCommandGroup {

    public AutoEject(Arm arm, Intake intake) {
        addRequirements(arm, intake);

        // Simple cone handoff...
        addCommands(new InstantCommand(() -> {
            intake.setPreset(IntakeConstants.PRESETS.CUBE); 
            intake.grabCube(0.6);
        }));
    
    }
    
}
