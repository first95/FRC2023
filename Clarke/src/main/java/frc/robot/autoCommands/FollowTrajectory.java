package frc.robot.autoCommands;

import java.util.function.Supplier;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.Auton;
import frc.robot.subsystems.SwerveBase;

public class FollowTrajectory extends SequentialCommandGroup{
    
    public FollowTrajectory(SwerveBase drivebase, PathPlannerTrajectory trajectory, boolean resetOdometry) {
        addRequirements(drivebase);

        // fix this
        if(resetOdometry) {
            drivebase.resetOdometry(trajectory.getInitialHolonomicPose());
        }
        
        addCommands(
            new PPSwerveControllerCommand(
                trajectory,
                drivebase::getPose,
                new PIDController(Auton.X_KP, Auton.ANG_KI, Auton.ANG_KD),
                new PIDController(Auton.Y_KP, Auton.Y_KI, Auton.Y_KD),
                new PIDController(Auton.ANG_KP, Auton.ANG_KI, Auton.ANG_KD),
                drivebase::setChassisSpeeds,
                drivebase)
        );
    }
    
    public FollowTrajectory(SwerveBase drivebase, Supplier<PathPlannerTrajectory> trajectoryGen, boolean resetOdometry) {
        addRequirements(drivebase);
        
        if(resetOdometry) {
            addCommands(new InstantCommand(() -> drivebase.resetOdometry(trajectoryGen.get().getInitialHolonomicPose())));
        }

        addCommands(new FunctionalSwerveController(
                trajectoryGen,
                drivebase::getPose,
                new PIDController(Auton.X_KP, Auton.X_KI, Auton.X_KD),
                new PIDController(Auton.Y_KP, Auton.Y_KI, Auton.Y_KD),
                new PIDController(Auton.ANG_KP, Auton.ANG_KI, Auton.ANG_KD),
                drivebase::setChassisSpeeds,
                drivebase));
        
    }
}
