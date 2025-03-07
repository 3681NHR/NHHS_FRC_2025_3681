package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.IntakeConstants;
import frc.robot.subsystems.intake.Intake;

public class IntakeCommand extends Command {

  private Intake intake;
  private boolean stopOnHold;

  public IntakeCommand(Intake intake) {
    this.intake = intake;
    addRequirements(intake);
  }

  @Override
  public void initialize() {
    //if not holding, stop when holding, otherwise dont stop
    this.stopOnHold = !intake.isHolding();
    if(!intake.getHoldLock()){
      this.stopOnHold = false;
    }
  }

  @Override
  public void execute() {
    intake.setVoltage(IntakeConstants.SPEED);
  }

  @Override
  public void end(boolean interrupted) {
    intake.stop();
  }

  @Override
  public boolean isFinished() {
    return stopOnHold && intake.isHolding();
  }
}
