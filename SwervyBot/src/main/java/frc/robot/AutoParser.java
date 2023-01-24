package frc.robot;

import java.util.Arrays;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutoParser {
    private CommandBase AutoMove;

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
                
                String function = stringCommand.split("(")[0];
                
                String[] parameters = stringCommand.split("(")[1].split(",");
                for (int j = 0; j < parameters.length; j++) {
                    parameters[j] = parameters[j].trim().replaceAll("[()]", "");
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
        return null;
    }


}
