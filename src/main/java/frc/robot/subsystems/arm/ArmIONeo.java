package frc.robot.subsystems.arm;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Constants.arm.*;
import frc.robot.Constants;
import frc.robot.Constants.arm;

public class ArmIONeo implements ArmIO{

    private SparkMax angle1 = new SparkMax(ids.ANGLE_1_ID, MotorType.kBrushless);
    private SparkMax angle2 = new SparkMax(ids.ANGLE_2_ID, MotorType.kBrushless);

    private SparkMax extention1 = new SparkMax(ids.EXTENTION_1_ID, MotorType.kBrushless);
    private SparkMax extention2 = new SparkMax(ids.EXTENTION_2_ID, MotorType.kBrushless);

    private SparkMax wrist = new SparkMax(ids.WRIST_ID, MotorType.kBrushless);

    private DutyCycleEncoder angleEncoder = new DutyCycleEncoder(ids.ANGLE_ENCODER);
    private DutyCycleEncoder extentionEncoder = new DutyCycleEncoder(ids.EXTENTION_ENCODER);
    private DutyCycleEncoder wristEncoder = new DutyCycleEncoder(ids.WRIST_ENCODER);

    private SparkMaxConfig angleConfig = new SparkMaxConfig();
    private SparkMaxConfig angleFollowConfig = new SparkMaxConfig();
    
    private SparkMaxConfig extentionConfig = new SparkMaxConfig();
    private SparkMaxConfig extentionFollowConfig = new SparkMaxConfig();

    private SparkMaxConfig wristConfig = new SparkMaxConfig();

    private ProfiledPIDController anglePID = new ProfiledPIDController( gains.ANGLE_KP,
                                                                        gains.ANGLE_KI,
                                                                        gains.ANGLE_KD,
                                                                        new Constraints(720, 720));
    private ArmFeedforward angleFF = new ArmFeedforward(gains.ANGLE_KS,
                                                        gains.ANGLE_KG,
                                                        gains.ANGLE_KV);

    private ProfiledPIDController extentionPID = new ProfiledPIDController( gains.EXTENTION_KP,
                                                                            gains.EXTENTION_KI,
                                                                            gains.EXTENTION_KD,
                                                                            new Constraints(100, 10));
    private ArmFeedforward extentionFF = new ArmFeedforward(gains.EXTENTION_KS,
                                                            gains.EXTENTION_KG,
                                                            gains.EXTENTION_KV);

    private ProfiledPIDController wristPID = new ProfiledPIDController( gains.WRIST_KP,
                                                                        gains.WRIST_KI,
                                                                        gains.WRIST_KD,
                                                                        new Constraints(800, 800));
    private ArmFeedforward wristFF = new ArmFeedforward(gains.WRIST_KS,
                                                        gains.WRIST_KG,
                                                        gains.WRIST_KV);

    private double angleCurrent;
    private double angleSetpoint;
    
    private double extentionCurrent;
    private double extentionSetpoint;

    private double wristCurrent;
    private double wristSetpoint;

    private Pose3d arm1pose = new Pose3d(arm.OFFSET, new Rotation3d());
    private Pose3d arm2pose = new Pose3d(arm.OFFSET, new Rotation3d());
    private Pose3d arm3pose = new Pose3d(arm.OFFSET, new Rotation3d());
    private Pose3d wristpose = new Pose3d(arm.OFFSET, new Rotation3d());

    public ArmIONeo(){
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

        wristConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(20)
            .voltageCompensation(12);

        angle1.configure(angleConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        angle2.configure(angleFollowConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        
        extention1.configure(extentionConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        extention2.configure(extentionFollowConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        wrist.configure(wristConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        angleEncoder.setInverted(gains.ANGLE_ENCODER_INVERTED);
        extentionEncoder.setInverted(gains.EXTENTION_ENCODER_INVERTED);
        wristEncoder.setInverted(gains.WRIST_ENCODER_INVERTED);
    }
    @Override
    public void updateInputs(ArmIOInputs input){
        updateEncoders();
        //move
        angle1.set(anglePID.calculate(angleCurrent, angleSetpoint) + angleFF.calculate(angleCurrent, anglePID.getSetpoint().velocity));
        extention1.set(extentionPID.calculate(extentionCurrent, extentionSetpoint) + extentionFF.calculate(extentionCurrent, extentionPID.getSetpoint().velocity));
        wrist.set(wristPID.calculate(wristCurrent, wristSetpoint) + wristFF.calculate(wristCurrent, wristPID.getSetpoint().velocity));
        
        input.armP1Pose = arm1pose;
        input.armP2Pose = arm2pose;
        input.armP3Pose = arm3pose;
        input.wristPose = wristpose;

        input.angleMotor1Temp = angle1.getMotorTemperature();
        input.angleMotor2Temp = angle2.getMotorTemperature();
        input.extentionMotor1Temp = extention1.getMotorTemperature();
        input.extentionMotor2Temp = extention2.getMotorTemperature();
        input.wristMotorTemp = wrist.getMotorTemperature();

        input.angleAppliedOut = angle1.getAppliedOutput();
        input.extentionAppliedOut = extention1.getAppliedOutput();
        input.wristAppliedOut = wrist.getAppliedOutput();

        input.angleCurrentDrawAmps = angle1.getOutputCurrent();
        input.extentionCurrentDrawAmps = extention1.getOutputCurrent();
        input.wristCurrentDrawAmps = wrist.getOutputCurrent();
    }
    @Override
    public void setAngleSetpoint(double setpoint) {
        angleSetpoint = setpoint;
    }

    @Override
    public void setExtentionSetpoint(double setpoint) {
        extentionSetpoint = setpoint;
    }

    @Override
    public void setWristSetpoint(double setpoint) {
        wristSetpoint = setpoint;
    }

    @Override
    public void setPositionSetpoint(ArmPosition a){
        setAngleSetpoint(a.angle);
        setExtentionSetpoint(a.extention);
        setWristSetpoint(a.wrist);
    }
    
    public void updateEncoders(){
        angleCurrent = angleEncoder.get()*gains.ANGLE_FACTOR + arm.ANGLE_OFFSET;
        extentionCurrent = extentionEncoder.get()*gains.EXTENTION_FACTOR + arm.EXTENTION_OFFSET;
        wristCurrent = wristEncoder.get()*gains.WRIST_FACTOR + arm.WRIST_OFFSET;
    }
    public void updateArmPose(){
        arm1pose = new Pose3d(arm.OFFSET.plus(new Translation3d(0, 0, Constants.robotDims.ROBOT_HEIGHT_OFF_GROUND)), new Rotation3d(0, -Units.degreesToRadians(angleSetpoint), 0));
        arm2pose = new Pose3d(arm.OFFSET.plus(new Translation3d(0, 0, Constants.robotDims.ROBOT_HEIGHT_OFF_GROUND)).plus(getArmPoint((extentionCurrent-positions.ARM_3_LENGTH)/2, angleCurrent)), new Rotation3d(0, -Units.degreesToRadians(angleCurrent), 0));
        arm3pose = new Pose3d(arm.OFFSET.plus(new Translation3d(0, 0, Constants.robotDims.ROBOT_HEIGHT_OFF_GROUND)).plus(getArmPoint(extentionCurrent-(positions.ARM_3_LENGTH), angleCurrent)), new Rotation3d(0, -Units.degreesToRadians(angleCurrent), 0));
        wristpose = new Pose3d(arm.OFFSET.plus(new Translation3d(0, 0, Constants.robotDims.ROBOT_HEIGHT_OFF_GROUND)).plus(getArmPoint(extentionCurrent, angleCurrent)), new Rotation3d(0, -Units.degreesToRadians(angleCurrent+wristCurrent), 0));            
    }
    
    private Translation3d getArmPoint(double dist, double angleDeg){
        double angleRad = Units.degreesToRadians(angleDeg);
        return new Translation3d(dist*Math.cos(angleRad), 0, dist*Math.sin(angleRad));
    }
}