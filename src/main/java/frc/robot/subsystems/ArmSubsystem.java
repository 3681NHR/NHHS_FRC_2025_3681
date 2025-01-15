package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.arm;
import frc.robot.Constants.arm.*;

public class ArmSubsystem extends SubsystemBase {

    private SparkMax angle1 = new SparkMax(ids.ANGLE_1_ID, MotorType.kBrushless);
    private SparkMax angle2 = new SparkMax(ids.ANGLE_2_ID, MotorType.kBrushless);

    private SparkMax extention1 = new SparkMax(ids.EXTENTION_1_ID, MotorType.kBrushless);
    private SparkMax extention2 = new SparkMax(ids.EXTENTION_2_ID, MotorType.kBrushless);

    private DutyCycleEncoder angleEncoder = new DutyCycleEncoder(ids.ANGLE_ENCODER);
    private DutyCycleEncoder extentionEncoder = new DutyCycleEncoder(ids.EXTENTION_ENCODER);

    private SparkMaxConfig angleConfig = new SparkMaxConfig();
    private SparkMaxConfig angleFollowConfig = new SparkMaxConfig();
    
    private SparkMaxConfig extentionConfig = new SparkMaxConfig();
    private SparkMaxConfig extentionFollowConfig = new SparkMaxConfig();

    private ProfiledPIDController anglePID = new ProfiledPIDController( gains.ANGLE_KP,
                                                                        gains.ANGLE_KI,
                                                                        gains.ANGLE_KD,
                                                                        null);
    private ArmFeedforward angleFF = new ArmFeedforward(gains.ANGLE_KS,
                                                        gains.ANGLE_KG,
                                                        gains.ANGLE_KV);

    private ProfiledPIDController extentionPID = new ProfiledPIDController( gains.EXTENTION_KP,
                                                                            gains.EXTENTION_KI,
                                                                            gains.EXTENTION_KD,
                                                        null);
    private ArmFeedforward extentionFF = new ArmFeedforward(gains.EXTENTION_KS,
                                                            gains.EXTENTION_KG,
                                                            gains.EXTENTION_KV);

    private double angleCurrent;
    private double angleSetpoint;
    
    private double extentionCurrent;
    private double extentionSetpoint;

    private State state = State.MAN;
    private Action action = Action.MOVING_TO_SETPOINT;

    public ArmSubsystem(){
        angleConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(40)
            .voltageCompensation(12);
        angleFollowConfig
            .apply(angleConfig)
            .follow(angle1);

        extentionConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(40)
            .voltageCompensation(12);
        extentionFollowConfig
            .apply(extentionConfig)
            .follow(extention1);

        angle1.configure(angleConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        angle2.configure(angleFollowConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        
        extention1.configure(extentionConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        extention2.configure(extentionFollowConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        angleEncoder.setInverted(gains.ANGLE_ENCODER_INVERTED);
        extentionEncoder.setInverted(gains.EXTENTION_ENCODER_INVERTED);
    }

    @Override
    public void periodic() {
        updateEncoders();
        updateSetpoints();
        if(RobotState.isDisabled()){
            angleSetpoint = angleCurrent;
        }


        angleSetpoint = MathUtil.clamp(angleSetpoint, limits.MIN_ANGLE, limits.MAX_ANGLE);
        //clamp extention within extention limit of robot perimiter, as defined in constants
        //if the arm is reaching out too much at min, womp womp
        double a =Constants.robotDims.ROBOT_SIZE.getY()/2;
        double allowedExtention = (a-(a-arm.OFFSET.getY())+a+limits.EXTENTION_LIMIT)/Math.cos(Units.degreesToRadians(angleCurrent));//dont change this math, it took me forever
        //clamp allowed extention so it will be a valid max
        allowedExtention = MathUtil.clamp(allowedExtention, limits.MIN_EXTENTION, limits.MAX_EXTENTION);
        extentionSetpoint = MathUtil.clamp(extentionSetpoint, limits.MIN_EXTENTION, allowedExtention);

        if(action != Action.DISABLED){
            //move
            angle1.set(anglePID.calculate(angleCurrent, angleSetpoint) + angleFF.calculate(angleCurrent, anglePID.getSetpoint().velocity));
            extention1.set(extentionPID.calculate(extentionCurrent, extentionSetpoint) + extentionFF.calculate(extentionCurrent, extentionPID.getSetpoint().velocity));
        }

    }
    public void updateEncoders(){
        angleCurrent = angleEncoder.get()*gains.ANGLE_FACTOR + arm.ANGLE_OFFSET;
        extentionCurrent = extentionEncoder.get()*gains.EXTENTION_FACTOR + arm.EXTENTION_OFFSET;
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
    public double getAngle(){return angleCurrent;}
    public double getExtention(){return extentionCurrent;}
    public Action getAction(){return action;}
    public State getState(){return state;}
    public void setAngle(double a){angleSetpoint = a;}
    public void setExtention(double e){extentionSetpoint = e;}
    public void setState(State s){state = s;}
    public void setAction(Action a){action = a;}
    public void setPosition(ArmPosition p){
        setAngle(p.angle);
        setExtention(p.extention);
    }
    
    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addStringProperty("mode", () -> state.toString(), null);
        builder.addStringProperty("state", () -> action.toString(), null);

        builder.addDoubleProperty("current angle", this::getAngle, null);
        builder.addDoubleProperty("target angle", () -> angleSetpoint, this::setAngle);

        builder.addDoubleProperty("current extention", this::getExtention, null);
        builder.addDoubleProperty("target extention", () -> angleSetpoint, this::setExtention);
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
        DISABLED//will not move
    }
}
