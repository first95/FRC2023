package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Arm;

public class ThrowCube extends SequentialCommandGroup {
    public ThrowCube(Arm arm) {
        addRequirements(arm);

        addCommands(new InstantCommand(() -> arm.setPos(0)));
        addCommands(new WaitUntilCommand(() -> arm.getPos() >= -60));
        addCommands(new InstantCommand(() -> arm.setGrip(true)));
    }
}
