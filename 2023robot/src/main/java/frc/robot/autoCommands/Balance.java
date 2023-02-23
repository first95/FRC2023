package frc.robot.autoCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.Auton;
import frc.robot.subsystems.SwerveBase;

public class Balance extends SequentialCommandGroup {

  public Balance(Alliance alliance, SwerveBase drive) {
    addCommands(new DriveToPose("StartNearBalance", alliance, drive));
    addCommands(new PrecisionAlign("ChargerCenter", alliance, drive));
    //addCommands(new PitchBalance(drive));
  }

  private class PitchBalance extends CommandBase {
      private final SwerveBase drive;
      private final Timer time;
      private final PIDController driveControl;
      private double pitch, lastPitch, lastTime, pitchRate, velocity;

      public PitchBalance(SwerveBase drive) {
          this.drive = drive;
          time = new Timer();
          driveControl = new PIDController(
            Auton.BALANCE_KP,
            Auton.BALANCE_KI,
            Auton.BALANCE_KD);
          driveControl.setSetpoint(0);
          driveControl.setTolerance(Auton.BALANCE_TOLERANCE);
          // Use addRequirements() here to declare subsystem dependencies.
          addRequirements(drive);
        }
      
        // Called when the command is initially scheduled.
        @Override
        public void initialize() {
          time.start();
          lastTime = time.get();
          lastPitch = drive.getPitch().getDegrees();
          pitchRate = 0;
        }
      
        // Called every time the scheduler runs while the command is scheduled.
        @Override
        public void execute() {
          pitch = drive.getPitch().getDegrees();
          velocity = MathUtil.clamp(
            driveControl.calculate(pitch),
            -Auton.BALANCE_MAX_SPEED,
            Auton.BALANCE_MAX_SPEED);
          drive.drive(
            new Translation2d(
              velocity,
              0),
            0,
            false,
            false);
          pitchRate = (pitch - lastPitch) / (time.get() - lastTime);
          lastTime = time.get();
          lastPitch = pitch;
        }
      
        // Called once the command ends or is interrupted.
        @Override
        public void end(boolean interrupted) {
          drive.drive(new Translation2d(), 0, true, true);
          drive.setDriveBrake();
          SmartDashboard.putString("Balance", "Done");
        }
      
        // Returns true when the command should end.
        @Override
        public boolean isFinished() {
          boolean approachingZeroFast = 
            ((pitch > 0) && (pitchRate < -Auton.BALANCE_ANG_RATE_LIMIT)) ||
            ((pitch < 0) && (pitchRate > Auton.BALANCE_ANG_RATE_LIMIT));
          return driveControl.atSetpoint() || approachingZeroFast;
        }
  }
}