package frc.robot.autoCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.Auton;
import frc.robot.subsystems.SwerveBase;

public class FinalBalance extends SequentialCommandGroup {
    private SwerveBase drive;
    public FinalBalance(SwerveBase drive) {
        this.drive = drive;
        addRequirements(drive);

        addCommands(new RepeatCommand(new PrecisionAlign(this::nudgePose, drive)).until(this::isLevel));
    }

    private Pose2d nudgePose() {
        Pose2d currentPose = drive.getPose();
        double[] gravVec = drive.getGravityVector();

        return null;
    }

    private boolean isLevel() {
        return (Math.abs(drive.getPitch().getDegrees()) < Auton.CHARGER_BALANCE_ANGLE_TOLERANCE);
    }
}