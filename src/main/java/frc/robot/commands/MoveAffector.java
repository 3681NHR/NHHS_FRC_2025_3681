package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.AffectorPosition;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.wrist.Wrist;

public class MoveAffector extends Command {

  private Elevator elevator;
  private Wrist wrist;

  private Supplier<AffectorPosition> pos;

  public MoveAffector(Elevator elevator, Wrist wrist, Supplier<AffectorPosition> pos) {
    this.elevator = elevator;
    this.wrist = wrist;
    this.pos = pos;
    addRequirements(elevator, wrist);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    elevator.setTargetPos(pos.get().elev);
    wrist.setPosSet(pos.get().wrist);
  }

  @Override
  public void end(boolean interrupted) {
    //elevator.setTargetPos(AffectorPosition.STOW.elev);
    //wrist.setPosSet(AffectorPosition.STOW.wrist);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
