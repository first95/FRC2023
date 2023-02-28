package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.Auton;
import frc.robot.autoCommands.PrecisionAlign;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.SwerveBase;

public class AutoScore extends SequentialCommandGroup {
    private final SwerveBase drive;
    private final Arm arm;
    private final Alliance alliance;
    public AutoScore(Supplier<Alliance> alliance, Arm arm, SwerveBase drive) {
        this.drive = drive;
        this.arm = arm;
        this.alliance = alliance.get();
        addCommands(new PrecisionAlign(this::pickNode, drive));
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

        ArmConstants.PRESETS nearestRow = ArmConstants.PRESETS.LOW_SCORE;
        double closestAngle = 360; // Arm can't get this far away from any of them
        double armPos = arm.getPos();
        ArmConstants.PRESETS[] scoreAngles = {
            ArmConstants.PRESETS.LOW_SCORE,
            ArmConstants.PRESETS.MID_SCORE,
            ArmConstants.PRESETS.HIGH_SCORE
        };
        for (ArmConstants.PRESETS preset : scoreAngles) {
            if (Math.abs(preset.angle() - armPos) < closestAngle) {
                nearestRow = preset;
            }
        }
        switch (nearestRow) {
            case HIGH_SCORE:
                nodeList = (gamepiece == "CONE") ? highConeNodes : highCubeNodes;
            break;
            case MID_SCORE:
                nodeList = (gamepiece == "CONE") ? midConeNodes : midCubeNodes;
            break;
            case LOW_SCORE:
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
