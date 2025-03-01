package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.AffectorPosition;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.wrist.Wrist;

public class MoveAffector extends Command {

  private Elevator elevator;
  private Wrist wrist;

  private AffectorPosition pos;

  public MoveAffector(Elevator elevator, Wrist wrist, AffectorPosition pos) {
    this.elevator = elevator;
    this.wrist = wrist;
    addRequirements(elevator, wrist);
    
    this.pos = pos;
  }

  @Override
  public void initialize() {
    elevator.setTargetPos(pos.elev);
    wrist.setPosSet(pos.wrist);
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
