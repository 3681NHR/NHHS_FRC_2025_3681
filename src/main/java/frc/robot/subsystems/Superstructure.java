package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.swerve.Drive;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.wrist.Wrist;

public class Superstructure extends SubsystemBase{
    public enum WantedSuperState {
        HOME,
        STOPPED,
        DEFAULT_STATE,
        INTAKE_CORAL,
        L4,
        L3,
        L2,
        L1,
        MANUAL
    }
    public enum CurrentSuperState {
        HOME,
        STOPPED,
        NO_PIECE_TELEOP,
        HOLDING_CORAL_TELEOP,
        NO_PIECE_AUTO,
        HOLDING_CORAL_AUTO,
        INTAKE_CORAL,
        L4,
        L3,
        L2,
        L1,
        MANUAL
    }

    private WantedSuperState wantedState    =  WantedSuperState.DEFAULT_STATE;
    private CurrentSuperState currentState  = CurrentSuperState.STOPPED;
    private CurrentSuperState previousState = CurrentSuperState.STOPPED;

    Intake intake;
    Climber climber;
    Elevator elevator;
    Drive drive;
    Vision vision;
    Wrist wrist;

    public Superstructure(){
        
    }

    @Override
    public void periodic() {
        previousState = currentState;
        
        Logger.recordOutput("Superstructure/previousState", previousState);
        Logger.recordOutput("Superstructure/currentState", currentState);
        Logger.recordOutput("Superstructure/wantedState", wantedState);
        
        stateTransition();
        applyStates();
        
    }

    //update current state
    private void stateTransition(){
        switch (wantedState) {
            case HOME:
                currentState = CurrentSuperState.HOME;
            break;
            case STOPPED:
                currentState = CurrentSuperState.STOPPED;
            break;
            case DEFAULT_STATE:
                if(DriverStation.isAutonomous()){
                    if(intake.isHolding()){
                        currentState = CurrentSuperState.HOLDING_CORAL_AUTO;
                    } else {
                        currentState = CurrentSuperState.NO_PIECE_AUTO;
                    }
                } else {
                    if(intake.isHolding()){
                        currentState = CurrentSuperState.HOLDING_CORAL_TELEOP;
                    } else {
                        currentState = CurrentSuperState.NO_PIECE_TELEOP;
                    }
                }
            break;
            case INTAKE_CORAL:
                currentState = CurrentSuperState.INTAKE_CORAL;
            break;
            case L4:
                currentState = CurrentSuperState.L4;
            break;
            case L3:
                currentState = CurrentSuperState.L3;
            break;
            case L2:
                currentState = CurrentSuperState.L2;
            break;
            case L1:
                currentState = CurrentSuperState.L1;
            break;
            case MANUAL:
                currentState = CurrentSuperState.MANUAL;
            break;
            default:
                currentState = CurrentSuperState.STOPPED;
            break;
        }
    }

    //apply current state
    private void applyStates(){

    }
}
