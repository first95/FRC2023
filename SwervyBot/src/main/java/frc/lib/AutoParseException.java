package frc.lib;

public class AutoParseException extends Exception {
    private String command;

    public AutoParseException(String command, String message) {
        super(message);
        this.command = command;
    }

    public AutoParseException(String command, String message, Throwable cause) {
        super(message, cause);
        this.command = command;
    }

    public String getCommand() {
        return command;
    }
}
