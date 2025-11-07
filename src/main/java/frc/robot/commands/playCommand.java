package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;

/**
 * A command that runs another command supplied by a {@link Supplier}. This is
 * useful for reinitalizing a command constructed before calling
 */
public class playCommand extends Command {
    private Supplier<Command> cmd;

    public playCommand(Supplier<Command> cmd) {
        this.cmd = cmd;

    }

    @Override
    public void initialize() {

        addRequirements(cmd.get().getRequirements());
        cmd.get().initialize();
    }

    @Override
    public void execute() {
        cmd.get().execute();
    }

    @Override
    public void end(boolean interrupted) {
        cmd.get().end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return cmd.get().isFinished();
    }
}
