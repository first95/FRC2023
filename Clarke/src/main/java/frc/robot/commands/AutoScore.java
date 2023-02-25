package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.autoCommands.PrecisionAlign;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.SwerveBase;

public class AutoScore extends SequentialCommandGroup {
    private final Constants.ROW row;
    public AutoScore(Constants.ROW row, Alliance alliance, Arm arm, SwerveBase drive) {
        this.row = row;
        addCommands(new PrecisionAlign(this::pickNode, alliance, drive));
        addCommands(new InstantCommand(arm::toggleGrip));
    }

    private String[] highNodes = {
        "Node1High",
        "Node2High",
        "Node3High",
        "Node4High",
        "Node5High",
        "Node6High",
        "Node7High",
        "Node8High",
        "Node9High"
   };
    private String[] midNodes = {
        "Node1Mid",
        "Node2Mid",
        "Node3Mid",
        "Node4Mid",
        "Node5Mid",
        "Node6Mid",
        "Node7Mid",
        "Node8Mid",
        "Node9Mid"
    };
    private String[] lowNodes = {
        "Node1Low",
        "Node2Low",
        "Node3Low",
        "Node4Low",
        "Node5Low",
        "Node6Low",
        "Node7Low",
        "Node8Low",
        "Node9Low"
    };
    private Pose2d pickNode() {
        String[] nodeList;
        switch (row) {
            case HIGH:
                nodeList = highNodes;
            break;
            case MIDDLE:
                nodeList = midNodes;
            break;
            case LOW:
                nodeList = lowNodes;
            break;
        }
        return null;
   }
}
