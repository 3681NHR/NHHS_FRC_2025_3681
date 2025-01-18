package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.arm;
import frc.robot.Constants.arm.*;
import frc.robot.subsystems.arm.ArmIOInputsAutoLogged;

public class ElevatorSubsystem extends SubsystemBase {

    private double angleCurrent;
    private double angleSetpoint;
    
    private double extentionCurrent;
    private double extentionSetpoint;

    private double wristCurrent;
    private double wristSetpoint;

    private State state = State.MAN;
    private Action action = Action.MOVING_TO_SETPOINT;


    private Elevator io;
    private ArmIOInputsAutoLogged input = new ArmIOInputsAutoLogged();

    private Pose3d targetArm1pose = new Pose3d(arm.OFFSET, new Rotation3d());
    private Pose3d targetArm2pose = new Pose3d(arm.OFFSET, new Rotation3d());
    private Pose3d targetArm3pose = new Pose3d(arm.OFFSET, new Rotation3d());
    private Pose3d targetWristpose = new Pose3d(arm.OFFSET, new Rotation3d());
   
    public ElevatorSubsystem(Elevator io){
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(input);
        Logger.processInputs("arm", input);

        angleCurrent = input.angle;
        extentionCurrent = input.extention;
        wristCurrent = input.wrist;

        updateSetpoints();
        updateAction();
        updatePose();

        if(RobotState.isDisabled()){
            angleSetpoint = angleCurrent;
            extentionSetpoint = extentionCurrent;
            wristSetpoint = wristCurrent;
        }

        angleSetpoint = MathUtil.clamp(angleSetpoint, limits.MIN_ANGLE, limits.MAX_ANGLE);
        wristSetpoint = MathUtil.clamp(wristSetpoint, limits.MIN_WRIST, limits.MAX_WRIST);
        //clamp extention within extention limit of robot perimiter, as defined in constants
        //if the arm is reaching out too much at min, womp womp
        //double a =Constants.robotDims.ROBOT_SIZE.getY()/2;
        //double allowedExtention = (a-(a-arm.OFFSET.getY())+a+limits.EXTENTION_LIMIT)/Math.cos(Units.degreesToRadians(angleCurrent));//dont change this math, it took me forever
        //clamp allowed extention so it will be a valid max
        //allowedExtention = MathUtil.clamp(allowedExtention, limits.MIN_EXTENTION, limits.MAX_EXTENTION);
        double allowedExtention = limits.MAX_EXTENTION;
        extentionSetpoint = MathUtil.clamp(extentionSetpoint, limits.MIN_EXTENTION, allowedExtention);

        io.setAngleSetpoint(getAngleSetpoint());
        io.setExtentionSetpoint(getExtentionSetpoint());
        io.setWristSetpoint(getWristSetpoint());

    }
    public void updateAction(){
        if(action != Action.DISABLED){
            action = Action.MOVING_TO_SETPOINT;
            if(MathUtil.isNear(extentionCurrent, extentionSetpoint, limits.EXTENTION_NEAR_RANGE)
            && MathUtil.isNear(angleCurrent, angleSetpoint, limits.ANGLE_NEAR_RANGE)
            && MathUtil.isNear(wristCurrent, wristSetpoint, limits.WRIST_NEAR_RANGE)){
                action = Action.NEAR_SETPOINT;
            }
            if(MathUtil.isNear(extentionCurrent, extentionSetpoint, limits.EXTENTION_DEADBAND)
            && MathUtil.isNear(angleCurrent, angleSetpoint, limits.ANGLE_DEADBAND)
            && MathUtil.isNear(wristCurrent, wristSetpoint, limits.WRIST_DEADBAND)){
                action = Action.HOLD_AT_SETPOINT;
            }
        }
    }
    public void updateSetpoints(){
        switch (state) {
            case CORAL_GROUND:
                setPosition(positions.CORAL_GROUND);
            return;
            case CORAL_STATION:
                setPosition(positions.CORAL_STATION);
            return;
            case CORAL_L1:
                setPosition(positions.CORAL_L1);
            return;
            case CORAL_L2:
                setPosition(positions.CORAL_L2);
            return;
            case CORAL_L3:
                setPosition(positions.CORAL_L3);
            return;
            case CORAL_L4:
                setPosition(positions.CORAL_L4);
            return;
            case ALGE_L2:
                setPosition(positions.ALGE_L2);
            return;
            case ALGE_L3:
                setPosition(positions.ALGE_L3);
            return;
            case PROSSESOR:
                setPosition(positions.PROSESSOR);
            return;
            default:
            break;
        }
    }
    public void updatePose(){
        
        targetArm1pose = new Pose3d(arm.OFFSET.plus(new Translation3d(0, 0, Constants.robotDims.ROBOT_HEIGHT_OFF_GROUND)), new Rotation3d(0, -Units.degreesToRadians(angleSetpoint), 0));
        targetArm2pose = new Pose3d(arm.OFFSET.plus(new Translation3d(0, 0, Constants.robotDims.ROBOT_HEIGHT_OFF_GROUND)).plus(getArmPoint((extentionSetpoint-positions.ARM_3_LENGTH)/2, angleSetpoint)), new Rotation3d(0, -Units.degreesToRadians(angleSetpoint), 0));
        targetArm3pose = new Pose3d(arm.OFFSET.plus(new Translation3d(0, 0, Constants.robotDims.ROBOT_HEIGHT_OFF_GROUND)).plus(getArmPoint(extentionSetpoint-(positions.ARM_3_LENGTH), angleSetpoint)), new Rotation3d(0, -Units.degreesToRadians(angleSetpoint), 0));
        targetWristpose = new Pose3d(arm.OFFSET.plus(new Translation3d(0, 0, Constants.robotDims.ROBOT_HEIGHT_OFF_GROUND)).plus(getArmPoint(extentionSetpoint, angleSetpoint)), new Rotation3d(0, -Units.degreesToRadians(angleSetpoint+wristSetpoint), 0));        

        Logger.recordOutput("arm/targetArmP1Pose", targetArm1pose);
        Logger.recordOutput("arm/targetArmP2Pose", targetArm2pose);
        Logger.recordOutput("arm/targetArmP3Pose", targetArm3pose);
        Logger.recordOutput("arm/targetWristPose", targetWristpose);
    }
    private Translation3d getArmPoint(double dist, double angleDeg){
        double angleRad = Units.degreesToRadians(angleDeg);
        return new Translation3d(dist*Math.cos(angleRad), 0, dist*Math.sin(angleRad));
    }
    
    public double getAngle(){return angleCurrent;}
    public double getAngleSetpoint(){return angleSetpoint;}
    public double getExtention(){return extentionCurrent;}
    public double getExtentionSetpoint(){return extentionSetpoint;}
    public double getWrist(){return wristCurrent;}
    public double getWristSetpoint(){return wristSetpoint;}
    public Action getAction(){return action;}
    public State getState(){return state;}
    public void setAngle(double a){angleSetpoint = a;}
    public void setExtention(double e){extentionSetpoint = e;}
    public void setWrist(double w){wristSetpoint = w;}
    public void setState(State s){state = s;}
    public void setAction(Action a){action = a;}
    public void setPosition(ElevatorPosition p){
        setAngle(p.angle);
        setExtention(p.extention);
        setWrist(p.wrist);
    }
    
    @Override
    public void initSendable(SendableBuilder builder) {
        //builder.addStringProperty("mode", () -> state.toString(), null);
        //builder.addStringProperty("state", () -> action.toString(), null);
//
        //builder.addDoubleProperty("current angle", this::getAngle, null);
        //builder.addDoubleProperty("target angle", () -> angleSetpoint, this::setAngle);
//
        //builder.addDoubleProperty("current extention", this::getExtention, null);
        //builder.addDoubleProperty("target extention", () -> extentionSetpoint, this::setExtention);
    //
        //builder.addDoubleProperty("current wrist angle", this::getWrist, null);
        //builder.addDoubleProperty("target wrist angle", () -> wristSetpoint, this::setWrist);
    }

    public enum State{
        CORAL_GROUND,
        CORAL_STATION,
        CORAL_L1,
        CORAL_L2,
        CORAL_L3,
        CORAL_L4,
        ALGE_L2,
        ALGE_L3,
        PROSSESOR,
        MAN
    }
    public enum Action{
        HOLD_AT_SETPOINT,//at setpoint deadband
        MOVING_TO_SETPOINT,//moving to setpoint
        NEAR_SETPOINT,//within near range of setpoint
        DISABLED,//will not move
        ENABLED//set to re enable, do not use as condition
    }
}