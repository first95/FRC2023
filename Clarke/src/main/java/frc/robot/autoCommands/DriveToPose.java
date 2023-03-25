package frc.robot.autoCommands;

import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.Auton;
import frc.robot.subsystems.SwerveBase;

public class DriveToPose extends SequentialCommandGroup{
    private SwerveBase drive;
    private Pose2d pose;
    private boolean startInMotion, endInMotion;
    
    /**
     * Creates a command to drive to any given pose in a straight line
     * by following a trajectory.  Should eventually take a named pose and alliance.
     * @param pose Pose2d to drive to.
     */
    public DriveToPose(String pose, boolean startInMotion, boolean endInMotion, Alliance alliance, SwerveBase drive) {
        this.drive = drive;
        this.pose = Auton.POSE_MAP.get(alliance).get(pose);
        this.startInMotion = startInMotion;
        this.endInMotion = endInMotion;
        addRequirements(drive);

        addCommands(new FollowTrajectory(drive, this::generateTrajectory, false));

        };

    private PathPlannerTrajectory generateTrajectory() {
        Pose2d currentPose = drive.getPose();
        Translation2d currentPosition = 
            new Translation2d(currentPose.getX(), currentPose.getY());
        Translation2d desiredPosition = 
            new Translation2d(pose.getX(), pose.getY());
        Translation2d deltaPosition = desiredPosition.minus(currentPosition);
        Rotation2d driveAngle = deltaPosition.getAngle();
        Rotation2d startAngle;
        double startVelocity, endVelocity;
        if (startInMotion) {
           ChassisSpeeds velocity = drive.getFieldVelocity();
           Translation2d roboVel = new Translation2d(velocity.vxMetersPerSecond, velocity.vyMetersPerSecond);
           startAngle = roboVel.getAngle();
           startVelocity = roboVel.getNorm();
        } else {
            startAngle = driveAngle;
            startVelocity = -1; // this is the default for some reason
        }
        if (endInMotion) {
            // Calculate maximum achievable velocity using Vf^2 = Vi^2 + 2ad
            // Min is to ensure this isn't greater than the limit
            endVelocity = Math.min(Auton.MAX_SPEED,
                Math.sqrt(Math.pow(startVelocity, 2) + (2 * Auton.MAX_ACCELERATION * deltaPosition.getNorm()))    
            ) * Auton.MAX_SPEED_SAFETY_SCALAR;
        } else {
            endVelocity = -1;
        }
        PathPlannerTrajectory trajectory = PathPlanner.generatePath(
            new PathConstraints(Auton.MAX_SPEED, Auton.MAX_ACCELERATION),
            List.of(
                new PathPoint(currentPosition, startAngle, drive.getPose().getRotation(), startVelocity),
                new PathPoint(desiredPosition, driveAngle, pose.getRotation(), endVelocity)
            ));
        return trajectory;
    }
}
