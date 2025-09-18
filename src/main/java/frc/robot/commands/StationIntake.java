package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.AffectorPosition;
import frc.robot.constants.Constants;
import frc.robot.constants.IntakeConstants;
import frc.robot.subsystems.affector.Affector;
import frc.robot.subsystems.affector.Affector.WantedAffectorState;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.WantedIntakeState;

public class StationIntake extends Command {

  private Affector affector;
  private Intake intake;

  public StationIntake(Affector affector, Intake intake) {
    this.affector = affector;
    this.intake = intake;

    addRequirements(affector, intake);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {

    affector.setWantedState(WantedAffectorState.POSITION, Constants.Affector.STATION_POSITION);

    intake.setWantedState(WantedIntakeState.INTAKE);
  }

  @Override
  public void end(boolean interrupted) {
    intake.setWantedState(WantedIntakeState.STOP);
  }

  @Override
  public boolean isFinished() {
    return intake.isHolding();
  }
}
