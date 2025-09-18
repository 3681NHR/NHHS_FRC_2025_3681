package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.WantedIntakeState;

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
    intake.setWantedState(WantedIntakeState.INTAKE);
  }

  @Override
  public void end(boolean interrupted) {
    intake.setWantedState(WantedIntakeState.STOP);
  }

  @Override
  public boolean isFinished() {
    return stopOnHold && intake.isHolding();
  }
}
