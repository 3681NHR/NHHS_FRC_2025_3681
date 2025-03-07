package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ElevatorConstants;
import frc.robot.subsystems.elevator.*;

public class HomeElevator extends Command{

    private final Elevator elevator;

    /**
     * The current timestamp in the zeroing process
     */
    private double zeroTimeStamp;

    /**
     * Zeros the arm so that all values are set relative to the arm.
     * @param elevator {@link ArmSubsystem}
     */
    public HomeElevator(Elevator armSubsystem) {
        this.elevator = armSubsystem;

        addRequirements(armSubsystem);
    }

    @Override
    public void initialize() {
        zeroTimeStamp = Double.NaN; // Initializing starting time stamp
        elevator.setHomed(false); // Setting zeroed field within the subsystem to be false.
        elevator.setTargetPos(elevator.getPosition());
        elevator.setVoltage(0);
    }

    @Override
    public void execute() {
        if (manageTimer(ElevatorConstants.HOME_MIN_VEL)) {
            elevator.setVoltage(0.0);
            elevator.resetPos(ElevatorConstants.HOME_POS);
            elevator.setHomed(true);
            zeroTimeStamp = Double.NaN;
        } else {
            elevator.setVoltage(ElevatorConstants.HOME_VOLTAGE);
        }
    }

    @Override
    public void end(boolean interrupted) {
        // Stopping the motors
        elevator.setVoltage(0.0);
        if (!interrupted) {
            elevator.setHomed(true);
        }
        elevator.setTargetPos(elevator.getPosition());
    }

    @Override
    public boolean isFinished() {
        return elevator.isHomed();
    }

    private boolean manageTimer(double velocityThreshold) {
        if (Math.abs(elevator.getVelocity()) < velocityThreshold) {
            if (!Double.isFinite(zeroTimeStamp)) {
                zeroTimeStamp = Logger.getTimestamp();
                return false;
            } else {
                return Logger.getTimestamp() - zeroTimeStamp >= Units.secondsToMilliseconds(ElevatorConstants.HOME_STOP_TIME);
            }
        } else {
            zeroTimeStamp = Double.NaN;
            return false;
        }
    }
}