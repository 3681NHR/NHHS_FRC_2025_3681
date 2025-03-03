package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.AffectorPosition;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.WristConstants;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.swerve.Drive;
import frc.robot.subsystems.wrist.Wrist;

//handles coordinated actions and controll for multiple subsystems
public class Superstructure extends SubsystemBase {
    
    public enum WantedState{
        STOW,
        PREP_L1,
        PREP_L2,
        PREP_L3,
        PREP_L4,
        SCORE_L1,
        SCORE_L2,
        SCORE_L3,
        SCORE_L4,
        STATION_INTAKE,
        STOPPED,
        MANUAL,
        OUTTAKE
    }
    public enum CurrentState{
        STOW,
        PREP_L1,
        PREP_L2,
        PREP_L3,
        PREP_L4,
        SCORE_L1,
        SCORE_L2,
        SCORE_L3,
        SCORE_L4,
        STATION_INTAKE,
        MANUAL,
        STOPPED,
        OUTTAKE
    }

    private Elevator elevator;
    private Wrist wrist;
    private Intake intake;
    private Drive drive;
    private Led led;
    //vision?

    private AffectorPosition targetPos = AffectorPosition.STOW;

    private WantedState wantedState = WantedState.STOPPED;
    private CurrentState currentState = CurrentState.STOPPED;

    public Superstructure(
        Elevator elevator,
        Wrist wrist,
        Intake intake,
        Drive drive,
        Led led
    ){
        this.elevator = elevator;
        this.wrist = wrist;
        this.intake = intake;
        this.drive = drive;
        this.led = led;
    }

    @Override
    public void periodic() {
        updateAScopePoses();
        updateLEDs();
        handleStateTransition();
        applyStates();

        Logger.recordOutput("Superstate/wantedState", wantedState);
        Logger.recordOutput("Superstate/currentState", currentState);
    }

    public void handleStateTransition(){
        switch (wantedState) {
            case STOW:
                currentState = CurrentState.STOW;
            break;
            case PREP_L1:
                currentState = CurrentState.PREP_L1;
            break;
            case PREP_L2:
                currentState = CurrentState.PREP_L2;
            break;
            case PREP_L3:
                currentState = CurrentState.PREP_L3;
            break;
            case PREP_L4:
                currentState = CurrentState.PREP_L4;
            break;
            case SCORE_L1:
                currentState = isReadyL1() ? CurrentState.SCORE_L1 : CurrentState.PREP_L1;
            break;
            case SCORE_L2:
                currentState = isReadyL2() ? CurrentState.SCORE_L2 : CurrentState.PREP_L2;
            break;
            case SCORE_L3:
                currentState = isReadyL3() ? CurrentState.SCORE_L3 : CurrentState.PREP_L3;
            break;
            case SCORE_L4:
                currentState = isReadyL4() ? CurrentState.SCORE_L4 : CurrentState.PREP_L4;
            break;
            case STATION_INTAKE:
                currentState = CurrentState.STATION_INTAKE;
            break;
            case STOPPED:
                currentState = CurrentState.STOPPED;
            break;

            case MANUAL:
                currentState = CurrentState.MANUAL;
            break;
            case OUTTAKE:
                currentState = CurrentState.OUTTAKE;
            break;
            default:
            break;
        }
    }

    public void applyStates(){
        switch (currentState) {
            case STOW:
                stow();
            break;
            case PREP_L1:
                handlePrepL1();
            break;
            case PREP_L2:
                handlePrepL2();
            break;
            case PREP_L3:
                handlePrepL3();
            break;
            case PREP_L4:
                handlePrepL4();
            break;
            case SCORE_L1:
                handleScore();
            break;
            case SCORE_L2:
                handleScore();
            break;
            case SCORE_L3:
                handleScore();
            break;
            case SCORE_L4:
                handleScore();
            break;
            case STATION_INTAKE:
                handleStationIntake();
            break;
            case STOPPED:
                handleStopped();
            break;

            case MANUAL:
                handleMan();
            break;
            case OUTTAKE:
                handleOuttake();
            break;
        
            default:
            break;
        }
    }

    public boolean isReadyL1(){
        return elevator.getPositionSet() == AffectorPosition.L1.elev && elevator.inPosition()
        && wrist.getPosSet() == AffectorPosition.L1.wrist && wrist.inPosition();
    }
    public boolean isReadyL2(){
        return elevator.getPositionSet() == AffectorPosition.L2.elev && elevator.inPosition()
        && wrist.getPosSet() == AffectorPosition.L2.wrist && wrist.inPosition();
    }
    public boolean isReadyL3(){
        return elevator.getPositionSet() == AffectorPosition.L3.elev && elevator.inPosition()
        && wrist.getPosSet() == AffectorPosition.L3.wrist && wrist.inPosition();
    }
    public boolean isReadyL4(){
        return elevator.getPositionSet() == AffectorPosition.L4.elev && elevator.inPosition()
        && wrist.getPosSet() == AffectorPosition.L4.wrist && wrist.inPosition();
    }
    public boolean isReadyStation(){
        return elevator.getPositionSet() == AffectorPosition.STATION.elev && elevator.inPosition()
        && wrist.getPosSet() == AffectorPosition.STATION.wrist && wrist.inPosition();
    }

    public boolean isReady(){
        return elevator.isHomed()//elevator homed
         && elevator.inPosition() && wrist.inPosition()//in pos
         && (targetPos.isScoring() ? intake.isHolding() || !intake.getHoldLock() : true)//holding if in scoring pos
         && (targetPos == AffectorPosition.STATION ? intake.isIntaking() : true);//intaking if in station pos
    }

    public void setTargetPos(AffectorPosition pos){
        targetPos = pos;
    }

    public void prepareScore(AffectorPosition pos){
        targetPos = pos;
        prepareScore();
    }

    public void prepareScore(){
        elevator.setTargetPos(targetPos.elev);
        wrist.setPosSet(targetPos.wrist);
    }

    public void updateAScopePoses(){
        Logger.recordOutput("componentPoses", new Pose3d[] {
            elevator.getAScopePoseMiddleStage(),
            elevator.getAScopePoseInnerStage(),
            wrist.getAScopePoseWrist(elevator.getPosition()),
            intake.isHolding() ? new Pose3d(
                WristConstants.WRIST_POS.plus(new Translation3d(0, Math.cos(wrist.getPos())*IntakeConstants.pivotToCoral, elevator.getPosition() + Math.sin(wrist.getPos())*IntakeConstants.pivotToCoral)),
                new Rotation3d(0, -wrist.getPos()+Math.PI/2, 0).rotateBy(new Rotation3d(0, 0, Math.PI/2))
            ) : new Pose3d(new Translation3d(0, 0, -10), new Rotation3d()),
        });
    }
    public void updateLEDs(){
        led.setColor(
            isReady() ? Color.kWhite
            : intake.isHolding() ? Color.kGreen : Color.kOrange
        );
        led.setHomed(elevator.isHomed());
        led.setIntaking(intake.isMoving());
    }

    public void handleScore(){
        intake.setVoltage(IntakeConstants.SPEED);
    }
    public void stow(){
        elevator.setTargetPos(AffectorPosition.STOW.elev);
        wrist.setPosSet(AffectorPosition.STOW.wrist);
        intake.stop();
    }
    public void handleStopped(){
        intake.stop();
        wrist.stop();
        elevator.stop();
    }
    public void handleStationIntake(){
        if(intake.isHolding() && intake.getHoldLock()){
            intake.stop();
        } else {
            intake.setVoltage(IntakeConstants.SPEED);
        }
    }
    public void handlePrepL1(){
        elevator.setTargetPos(AffectorPosition.L1.elev);
        wrist.setPosSet(AffectorPosition.L1.wrist);
    }
    public void handlePrepL2(){
        elevator.setTargetPos(AffectorPosition.L2.elev);
        wrist.setPosSet(AffectorPosition.L2.wrist);
    }
    public void handlePrepL3(){
        elevator.setTargetPos(AffectorPosition.L3.elev);
        wrist.setPosSet(AffectorPosition.L3.wrist);
    }
    public void handlePrepL4(){
        elevator.setTargetPos(AffectorPosition.L4.elev);
        wrist.setPosSet(AffectorPosition.L4.wrist);
    }
    public void handleOuttake(){
        intake.setVoltage(-IntakeConstants.SPEED);
    }
    public void handleMan(){

    }
    public void setWantedState(WantedState w){
        wantedState = w;
    }
}
