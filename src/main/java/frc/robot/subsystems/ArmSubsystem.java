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
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotState;
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

    public ArmSubsystem(){
        angleConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(40);
        angleFollowConfig
            .apply(angleConfig)
            .follow(angle1);

        extentionConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(40);
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
        if(RobotState.isDisabled()){
            angleSetpoint = angleCurrent;
        }

        angleSetpoint = MathUtil.clamp(angleSetpoint, limits.MIN_ANGLE, limits.MAX_ANGLE);

        //clamp extention within extention limit of robot perimiter, as defined in constants
        double a =Constants.robotDims.ROBOT_SIZE.getY()/2;
        extentionSetpoint = MathUtil.clamp(extentionSetpoint, limits.MIN_EXTENTION, (a-(a-arm.OFFSET.getY())+a+limits.EXTENTION_LIMIT)/Math.cos(Units.degreesToRadians(angleCurrent)));
        extentionSetpoint = MathUtil.clamp(extentionSetpoint, limits.MIN_EXTENTION, limits.MAX_EXTENTION);

        angle1.set(anglePID.calculate(angleCurrent, angleSetpoint) + angleFF.calculate(angleCurrent, 0));//TODO: ff velocity setpoint
    }
    public void updateEncoders(){
        angleCurrent = angleEncoder.get()*gains.ANGLE_FACTOR + arm.ANGLE_OFFSET;
        extentionCurrent = extentionEncoder.get()*gains.EXTENTION_FACTOR + arm.EXTENTION_OFFSET;
    }
}
