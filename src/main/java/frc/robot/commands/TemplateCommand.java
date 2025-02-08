package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

/**
 * template command
 * copy this file for commands
 * so you dont have to write all of this again
 * ;)
 */
public class TemplateCommand extends Command {

  /**
   * called when command is first made
   * <p>takes all subsystems used as params
   * should call addRequirements() for each subsystem 
   * to avoid multiple commands using the same subsystem
   */
  public TemplateCommand() {
    
  }

  /**
   * called when command is first called
   */
  @Override
  public void initialize() {}

  /**
   * called every 20ms the command is running
   */
  @Override
  public void execute() {}

  /**
   * called when command ends from isFinished or is interupted
   */
  @Override
  public void end(boolean interrupted) {}

  /**
   * get if the command is done
   * <p>if true, will call end() and stop the command
   */
  @Override
  public boolean isFinished() {
    return false;
  }
}
