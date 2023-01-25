package frc.robot;

import java.util.Arrays;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.Auton;
import frc.robot.autoCommands.FollowTrajectory;
import frc.robot.subsystems.SwerveBase;

public class AutoParser {
    private SwerveBase drive;
    private CommandBase AutoMove;

    public AutoParser(SwerveBase drive) {
        this.drive = drive;
    }

    public void parse(String autoMove, Alliance alliance) {
        String[] lines = autoMove.split(";");
        
        CommandBase[] commands = new CommandBase[lines.length];

        int lineNum = 0;
        for (String line : lines) {
            line = line.trim().toLowerCase();
            
            String[] stringCommands = line.split("\\+");

            CommandBase[] lineCommands = new CommandBase[stringCommands.length];
            
            int i = 0;
            for (String stringCommand : stringCommands) {
                stringCommand = stringCommand.trim();
                
                String[] splitCommand = stringCommand.split("\\(");

                String function = splitCommand[0];
                
                String[] parameters = splitCommand[1].split(",");
                for (int j = 0; j < parameters.length; j++) {
                    parameters[j] = parameters[j].trim().replaceAll("[\\(\\)]", "");
                }

                lineCommands[i] = stringParser(function, parameters);

                i++;
            }

            CommandBase lineCommand = lineCommands[0].alongWith(Arrays.copyOfRange(lineCommands, 1, lineCommands.length));

            commands[lineNum] = lineCommand;

            lineNum++;
        }

        AutoMove = commands[0].andThen(Arrays.copyOfRange(commands, 1, commands.length));
    }

    public CommandBase getAutoCommand() {
        return AutoMove;
    }

    private CommandBase stringParser(String command, String[] parameters) {
        System.out.println(command);
        System.out.println(parameters);
        switch (command) {
            case "followtrajectory":
                PathPlannerTrajectory trajectory = PathPlanner.loadPath(parameters[0], 
                    new PathConstraints(Auton.MAX_SPEED, Auton.MAX_ACCELERATION));
                return new FollowTrajectory(drive, trajectory, Boolean.valueOf(parameters[1]));
            case "wait":
                return new WaitCommand(Double.parseDouble(parameters[0]));
            case "stop":
                return new InstantCommand(drive::setDriveBrake);
            default:
                return new InstantCommand();
        }
    }


}
