package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.Auton;
import frc.robot.autoCommands.PrecisionAlign;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.SwerveBase;

public class AutoScore extends SequentialCommandGroup {
    private final Constants.ROW row;
    private final SwerveBase drive;
    private final Alliance alliance;
    public AutoScore(Constants.ROW row, Alliance alliance, Arm arm, SwerveBase drive) {
        this.drive = drive;
        this.row = row;
        this.alliance = alliance;
        addCommands(new PrecisionAlign(this::pickNode, alliance, drive));
        addCommands(new InstantCommand(arm::toggleGrip));
    }

    private String[] highConeNodes = {
        "Node1High",
        "Node3High",
        "Node4High",
        "Node6High",
        "Node7High",
        "Node9High"
   };
   private String[] highCubeNodes = {
    "Node2High",
    "Node5High",
    "Node8High"
   };
    private String[] midConeNodes = {
        "Node1Mid",
        "Node3Mid",
        "Node4Mid",
        "Node6Mid",
        "Node7Mid",
        "Node9Mid"
    };
    private String[] midCubeNodes = {
        "Node2Mid",
        "Node5Mid",
        "Node8Mid"
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
        String gamepiece = SmartDashboard.getString("LastHandoff", "CONE");
        switch (row) {
            case HIGH:
                nodeList = (gamepiece == "CONE") ? highConeNodes : highCubeNodes;
            break;
            case MIDDLE:
                nodeList = (gamepiece == "CONE") ? midConeNodes : midCubeNodes;
            break;
            case LOW:
                nodeList = lowNodes;
            break;
            default:
                nodeList = lowNodes;
        }
        Pose2d closest = Auton.POSE_MAP.get(alliance).get(nodeList[0]);
        double closestDistance = Double.POSITIVE_INFINITY;
        Pose2d currentPose = drive.getPose();
        for (String node : nodeList) {
            Pose2d pose = Auton.POSE_MAP.get(alliance).get(node);
            double distance = 
                currentPose.getTranslation().getDistance(
                    pose.getTranslation()
                );
            if (distance < closestDistance) {
                closestDistance = distance;
                closest = pose;
            }
        }
        return closest;
   }
}
