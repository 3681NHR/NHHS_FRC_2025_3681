package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.AffectorPosition;
import frc.robot.constants.Constants;
import frc.robot.constants.ElevatorConstants;
import frc.robot.subsystems.affector.Affector;
import frc.robot.subsystems.affector.Affector.WantedState;

public class MoveAffector extends Command {

  private Affector elevator;

  private boolean done = false;

  private AffectorPosition pos;
  private Supplier<AffectorPosition> posSup;

  public MoveAffector(Affector elevator, Supplier<AffectorPosition> pos) {
    this.elevator = elevator;
    this.posSup = pos;
    addRequirements(elevator);
  }
  @Override
  public String getName(){return "move affector";}

  @Override
  public void initialize() {
    pos = posSup.get();

    elevator.setWantedState(WantedState.POSITION, new AffectorPosition(pos.elev, Constants.Affector.STOW_POSITION.wrist));

    done = false;
  }

  @Override
  public void execute() {
    if(Math.abs(elevator.getPosition().elev - elevator.getPositionSet().elev) < ElevatorConstants.NEAR_POS_TOLERANCE){
      elevator.setWantedState(WantedState.POSITION, pos);
      done = elevator.atSetpoint();
    }
  }

  @Override
  public void end(boolean interrupted) {
    //elevator.setTargetPos(elevator.getPosition());
    //wrist.setPosSet(wrist.getPos());;
  }

  @Override
  public boolean isFinished() {
    return done;//TODO: end on signifacant operator input
  }
}
