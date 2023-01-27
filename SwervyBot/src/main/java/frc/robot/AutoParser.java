package frc.robot;

import java.util.Arrays;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.lib.AutoParseException;
import frc.robot.Constants.Auton;
import frc.robot.autoCommands.FollowTrajectory;
import frc.robot.subsystems.SwerveBase;

public class AutoParser {
    private SwerveBase drive;
    private CommandBase AutoMove;

    public AutoParser(SwerveBase drive) {
        this.drive = drive;
    }

    public String parse(String autoMove, Alliance alliance) throws Exception {
        System.out.println(autoMove);
        String[] lines = autoMove.split(";");
        
        CommandBase[] commands = new CommandBase[lines.length];

        int lineNum = 0;
        for (String line : lines) {
            System.out.println(line);
            line = line.trim();
            
            String[] stringCommands = line.split("\\+");

            CommandBase[] lineCommands = new CommandBase[stringCommands.length];
            
            int i = 0;
            for (String stringCommand : stringCommands) {
                stringCommand = stringCommand.trim();
                System.out.println(stringCommand);
                
                String[] splitCommand = stringCommand.split("\\(");

                String function = splitCommand[0].toLowerCase();
                System.out.println(function);
                
                String[] parameters;
                try {
                    parameters = splitCommand[1].split(",");
                } catch (IndexOutOfBoundsException e) {
                    throw new Exception("Missing parenthases at line: " + i, e);
                }
                for (int j = 0; j < parameters.length; j++) {
                    parameters[j] = parameters[j].trim().replaceAll("[\\(\\)]", "");
                }

                try {
                    lineCommands[i] = stringParser(function, parameters);
                } catch (AutoParseException e) {
                    throw new Exception(String.format("Failed to parse %s at line %d: %s", e.getCommand(), lineNum, e.getMessage()), e);
                }

                i++;
            }

            CommandBase lineCommand = lineCommands[0].alongWith(Arrays.copyOfRange(lineCommands, 1, lineCommands.length));

            commands[lineNum] = lineCommand;

            lineNum++;
        }

        AutoMove = commands[0].andThen(Arrays.copyOfRange(commands, 1, commands.length));
        
        return "Parsed Successfully!";
    }

    public CommandBase getAutoCommand() {
        return AutoMove;
    }

    private CommandBase stringParser(String command, String[] parameters) throws AutoParseException {
        System.out.println(command);
        System.out.println(parameters);
        switch (command) {
            case "followtrajectory":
                PathPlannerTrajectory trajectory = PathPlanner.loadPath(parameters[0], 
                    new PathConstraints(Auton.MAX_SPEED, Auton.MAX_ACCELERATION));
                try {
                    return new FollowTrajectory(drive, trajectory, Boolean.valueOf(parameters[1]));
                } catch (NullPointerException e) {
                    throw new AutoParseException("FollowTrajectory", "Path file not found", e);
                } catch (ArrayIndexOutOfBoundsException e) {
                    throw new AutoParseException("FollowTrajectory", "Missing parameters", e);
                }
            case "wait":
                try {
                    return new WaitCommand(Double.parseDouble(parameters[0]));
                } catch (NumberFormatException e) {
                    throw new AutoParseException("Wait", "Argument must be a number", e);
                }
            case "stop":
                try {
                    return new InstantCommand(drive::setDriveBrake);
                } catch (Exception e) {
                    throw new AutoParseException("Stop", "What did you do!?", e);
                }
            default:
                throw new AutoParseException("Unrecognized Command", "Command is not valid");
        }
    }


}
