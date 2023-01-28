package frc.robot.autoCommands;

import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.Auton;
import frc.robot.subsystems.SwerveBase;

public class DriveToPose extends SequentialCommandGroup{
    private SwerveBase drive;
    private Pose2d pose;
    
    /**
     * Creates a command to drive to any given pose in a straight line
     * by following a trajectory.  Should eventually take a named pose and alliance.
     * @param pose Pose2d to drive to.
     */
    public DriveToPose(Pose2d pose, SwerveBase drive) {
        this.drive = drive;
        this.pose = pose;
        addRequirements(drive);

        addCommands(new FollowTrajectory(drive, this::generateTrajectory, false));

        };

    private PathPlannerTrajectory generateTrajectory() {
        Pose2d currentPose = drive.getPose();
            Translation2d currentPosition = 
                new Translation2d(currentPose.getX(), currentPose.getY());
            Translation2d desiredPosition = 
                new Translation2d(pose.getX(), pose.getY());
            Rotation2d driveAngle = desiredPosition.minus(currentPosition).getAngle();
            PathPlannerTrajectory trajectory = PathPlanner.generatePath(
                new PathConstraints(Auton.MAX_SPEED, Auton.MAX_ACCELERATION),
                List.of(
                    new PathPoint(currentPosition, driveAngle, drive.getPose().getRotation()),
                    new PathPoint(desiredPosition, driveAngle, pose.getRotation())
                ));
        return trajectory;
    }
}
