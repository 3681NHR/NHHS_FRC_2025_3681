package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.AffectorPosition;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.wrist.Wrist;

public class MoveAffector extends Command {

  private Elevator elevator;
  private Wrist wrist;

  private boolean done = false;

  private AffectorPosition pos;
  private Supplier<AffectorPosition> posSup;

  public MoveAffector(Elevator elevator, Wrist wrist, Supplier<AffectorPosition> pos) {
    this.elevator = elevator;
    this.wrist = wrist;
    this.posSup = pos;
    addRequirements(elevator, wrist);
  }
  @Override
  public String getName(){return "move affector";}

  @Override
  public void initialize() {
    pos = posSup.get();

    wrist.setPosSet(AffectorPosition.STOW.wrist);
  
    elevator.setTargetPos(pos.elev);
    done = false;
  }

  @Override
  public void execute() {
    if(elevator.nearPos()){
      wrist.setPosSet(pos.wrist);
      done = elevator.inPosition() && wrist.inPosition();
    }
  }

  @Override
  public void end(boolean interrupted) {
    //elevator.setTargetPos(elevator.getPosition());
    //wrist.setPosSet(wrist.getPos());;
  }

  @Override
  public boolean isFinished() {
    return done || false;//TODO: end on signifacant operator input
  }
}
