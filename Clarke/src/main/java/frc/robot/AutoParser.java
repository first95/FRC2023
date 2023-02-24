package frc.robot;

import java.util.Arrays;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.Auton;
import frc.robot.Constants.IntakeConstants;
import frc.robot.autoCommands.DriveToPose;
import frc.robot.autoCommands.FollowTrajectory;
import frc.robot.autoCommands.PrecisionAlign;
import frc.robot.commands.AutoHandoffCone;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.SwerveBase;

public class AutoParser {
    private SwerveBase drive;
    private Arm arm;
    private Intake intake;
    private CommandBase AutoMove;
    private Alliance currentAlliance;

    /**
     * Creates an AutoParser for building an automove command from a DSL string.
     * Language specification is as follows:
     * <p>
     * Commands are case insensitive and must be followed by parenthases "()".  Any parameters
     * go in the parenthases and are comma-seperated.  Note that parameters are case sensitive.
     * A command must be followed by either a semicolon ";" or plus sign "+".  If followed by a
     * semicolon, the next command will run after the current command.  If followed by a plus
     * sign, the next command will run at the same time as the current command.
     * </p> 
     * Example:
     * <p><pre>
     * FollowTrajectory(firstPath, false) + RaiseArm(high);
     * ScoreObject();
     * followtrajectory(toRamp, false);
     * Balance();
     * </pre></p>
     * This will follow the trajectory named "firstPath" while raising an arm, then score, then follow
     * the trajectory named "toRamp", then balance.
     * <p>
     * Current commands are:
     * <pre>
     * FollowTrajectory(path, resetOdometry)
     * Wait(seconds)
     * Stop()
     * </pre></p>
     * @param drive The robot's drivebase subsystem
     */
    public AutoParser(SwerveBase drive, Arm arm, Intake intake) {
        this.drive = drive;
        this.arm = arm;
        this.intake = intake;
    }

    /**
     * Compile an automove string into a command that can be run on the robot.
     * @param autoMove The string of commands
     * @param alliance Which alliance the robot is on.  Required to correct for field mirroring.
     * @return A message stating compilation success
     * @throws Exception When compilation fails due to a source error.  Provides information 
     * about the error.
     */
    public String parse(String autoMove, Alliance alliance) throws Exception {
        currentAlliance = alliance;
        // Split the source string on semicolons, creating an array of
        // string-commands that must be run sequentially.
        String[] lines = autoMove.split(";");
        
        // Will be populated with the parsed commands
        CommandBase[] commands = new CommandBase[lines.length];

        // A foreach loop is more convenient, but index is also needed.
        int lineNum = 0;
        for (String line : lines) {
            // Remove whitespace, including newlines
            line = line.trim();
            
            // Split the line on plus signs, creating an array of
            // string-commands that must be run in parallel.
            String[] stringCommands = line.split("\\+");

            // Will be populated with the line's parsed commands
            CommandBase[] lineCommands = new CommandBase[stringCommands.length];
            
            // Again, foreach and index
            int i = 0;
            for (String stringCommand : stringCommands) {
                // Remove whitespace from around each string-command
                stringCommand = stringCommand.trim();
                
                // Split on open parenthases, to separate the command from the parameters
                String[] splitCommand = stringCommand.split("\\(");

                // Make commands case-insensitive by converting everything to lower case
                String function = splitCommand[0].toLowerCase();
                
                String[] parameters;
                // Split the parameter string on commas, to get a list of parameters.
                // This can cause an out-of-bounds exception if parenthases are missing,
                // so it is in a try block.
                try {
                    parameters = splitCommand[1].split(",");
                } catch (IndexOutOfBoundsException e) {
                    throw new Exception("Missing parenthases at line: " + i, e);
                }

                // Remove whitespace and the final closed parenthases from the parameters
                for (int j = 0; j < parameters.length; j++) {
                    parameters[j] = parameters[j].trim().replaceAll("[\\)]", "");
                }

                // The current string-command has now been seperated and cleaned, so we can attempt to parse it.
                // Again, this can cause an exception if the source string is incorrect.
                try {
                    lineCommands[i] = stringParser(function, parameters);
                } catch (AutoParseException e) {
                    // The AutoParseException contains information about what failed and why
                    throw new Exception(String.format("Failed to parse %s at line %d: %s", e.getCommand(), lineNum, e.getMessage()), e);
                }

                i++;
            }

            // Combine all commands that were seperated by a plus sign into a single parallel command group
            CommandBase lineCommand = lineCommands[0].alongWith(Arrays.copyOfRange(lineCommands, 1, lineCommands.length));

            // Add the command group to the array of line-commands
            commands[lineNum] = lineCommand;

            lineNum++;
        }

        // Combine all semicolon-seperated commands into a single sequential command group
        AutoMove = commands[0].andThen(Arrays.copyOfRange(commands, 1, commands.length));

        AutoMove.addRequirements(drive);
        
        return "Parsed Successfully!";
    }

    /**
     * Returns the parsed command
     * @return The command created by {@link AutoParser#parse()}
     */
    public CommandBase getAutoCommand() {
        return AutoMove;
    }

    /**
     * Parses a given string-command and parameters, returning a Command that can be run.
     * @param command The string-command
     * @param parameters An array of string parameters
     * @return A WPILib command matching the given command
     * @throws AutoParseException If parsing fails.  Contains information about the source errror.
     */
    private CommandBase stringParser(String command, String[] parameters) throws AutoParseException {
        switch (command) {
            case "followtrajectory":
            // Load the trajectory file
                PathPlannerTrajectory trajectory = PathPlanner.loadPath(parameters[0], 
                    new PathConstraints(Auton.MAX_SPEED, Auton.MAX_ACCELERATION));
                try {
                    trajectory = PathPlannerTrajectory.transformTrajectoryForAlliance(trajectory, currentAlliance);
                    return new FollowTrajectory(drive, trajectory, Boolean.valueOf(parameters[1]));
                } catch (NullPointerException e) {
                    // FollowTrajectory throws a NullPointerException if the trajectory is null, which occurs if the import fails.
                    throw new AutoParseException("FollowTrajectory", "Path file not found", e);
                } catch (ArrayIndexOutOfBoundsException e) {
                    // Would be triggered by parameters[1] if only one parameter was provided
                    throw new AutoParseException("FollowTrajectory", "Missing parameters", e);
                }
            case "wait":
                try {
                    return new WaitCommand(Double.parseDouble(parameters[0]));
                } catch (NumberFormatException e) {
                    throw new AutoParseException("Wait", "Argument must be a number", e);
                } catch (ArrayIndexOutOfBoundsException e) {
                    throw new AutoParseException("Wait", "Missing parameters", e);
                }
            case "stop":
                try {
                    return new InstantCommand(drive::setDriveBrake, drive);
                } catch (Exception e) {
                    throw new AutoParseException("Stop", "What did you do!?", e);
                }
            case "drivetopose":
                try {
                    return new DriveToPose(parameters[0], currentAlliance, drive);
                } catch (NullPointerException e) {
                    throw new AutoParseException("DriveToPose", "Pose not recognized", e);
                } catch (ArrayIndexOutOfBoundsException e) {
                    throw new AutoParseException("DriveToPose", "Missing parameters", e);
                }
            case "alignto":
                try {
                    return new PrecisionAlign(parameters[0], currentAlliance, drive);
                } catch (NullPointerException e) {
                    throw new AutoParseException("AlignTo", "Pose not recognized", e); 
                } catch (ArrayIndexOutOfBoundsException e) {
                    throw new AutoParseException("PrecisionAlign", "Missing parameters", e);
                }
            case "movearm":
                try {
                    return new InstantCommand(() -> arm.setPreset(ArmConstants.PRESETS.valueOf(parameters[0])), arm);
                } catch (ArrayIndexOutOfBoundsException e) {
                    throw new AutoParseException("MoveArm", "Missing parameters", e);
                } catch (IllegalArgumentException e) {
                    throw new AutoParseException("MoveArm", "Invalid preset position", e);
                }
            case "score":
                try {
                    return new InstantCommand(() -> arm.setGrip(true));
                } catch (Exception e) {
                    throw new AutoParseException("Score", "What did you do!?", e);
                }
            case "conehandoff":
                try {
                    return new AutoHandoffCone(arm, intake);
                } catch (Exception e) {
                    throw new AutoParseException("ConeHandoff", "What did you do!?", e);
                }
            case "grabcone":
                try {
                    return new InstantCommand(() -> {
                        intake.setPreset(IntakeConstants.PRESETS.CONE);
                        intake.grabCone(0.6);
                    }, intake)
                    .andThen(new WaitUntilCommand(() -> (intake.getTopRollerCurrentDraw() > IntakeConstants.GRABBED_CONE_ROLLER_CURRENT)))
                    .andThen(new InstantCommand(() -> {
                        intake.setPreset(IntakeConstants.PRESETS.HANDOFF);
                        intake.grabCone(0);
                    }, intake));
                } catch (Exception e) {
                    throw new AutoParseException("GrabCone", "What did you do!?", e);
                }
            default:
                // If none of the preceeding cases apply, the command is invalid.
                throw new AutoParseException("Unrecognized Command", "Command is not valid");
        }
    }

    private class AutoParseException extends Exception {
        private String command;
    
        /**
         * A custom exception for the AutoParser to use internally.
         * Contains the name of the command that failed to parse.
         */
        public AutoParseException(String command, String message) {
            super(message);
            this.command = command;
        }
    
        /**
         * A custom exception for the AutoParser to use internally.
         * Contains the name of the command that failed to parse.
         */
        public AutoParseException(String command, String message, Throwable cause) {
            super(message, cause);
            this.command = command;
        }
    
        /**
         * Gets the name of the command at fault
         * @return The command name
         */
        public String getCommand() {
            return command;
        }
    }
}
