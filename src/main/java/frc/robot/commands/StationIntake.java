package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.AffectorPosition;
import frc.robot.constants.IntakeConstants;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.wrist.Wrist;

public class StationIntake extends Command {

  private Elevator elevator;
  private Wrist wrist;
  private Intake intake;

  private double timeout = 0;
  private double startTime = Double.NaN;
  private boolean stopOnHold = true;

  public StationIntake(Elevator elevator, Wrist wrist, Intake intake) {
    this.elevator = elevator;
    this.wrist = wrist;
    this.intake = intake;

    addRequirements(elevator, wrist, intake);
  }

  @Override
  public void initialize() {
    stopOnHold = !intake.isHolding();
    startTime = Logger.getTimestamp();
  }

  @Override
  public void execute() {
    timeout = startTime-Logger.getTimestamp();

    elevator.setTargetPos(AffectorPosition.STATION.elev);
    wrist.setPosSet(AffectorPosition.STATION.wrist);

    intake.setVoltage(IntakeConstants.SPEED);
  }

  @Override
  public void end(boolean interrupted) {
    intake.setVoltage(0);
  }

  @Override
  public boolean isFinished() {
    return intake.isHolding() && stopOnHold || timeout >= 60;
  }
}
