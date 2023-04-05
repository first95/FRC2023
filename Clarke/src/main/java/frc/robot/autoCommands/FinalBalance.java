package frc.robot.autoCommands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Auton;
import frc.robot.subsystems.SwerveBase;

public class FinalBalance extends CommandBase {
    private final SwerveBase drive;
    private final Timer time;
    public FinalBalance(SwerveBase drive) {
        this.drive = drive;
        time = new Timer();
        addRequirements(drive); 
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        
        if (Math.abs(drive.getPitch().getDegrees()) < Auton.CHARGER_STARTING_TO_TIP) {
            time.start();
            drive.drive(new Translation2d(), 0, true, false);
        } else {
            time.stop();
            time.reset();
            double[] gravVec = drive.getGravityVector();;
            double xVel = -Math.copySign(Auton.BALANCE_SPEED, gravVec[0]);
            drive.drive(
                new Translation2d(xVel, 0), 0, true, false
        );
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        drive.drive(
            new Translation2d(), 
            0, 
            true,
            false);
    }

    @Override
    public boolean isFinished() {
        return (time.get() >= Auton.BALANCE_LEVEL_TIME);
    }
}