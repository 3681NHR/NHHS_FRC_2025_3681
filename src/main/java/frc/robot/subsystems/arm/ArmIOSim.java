package frc.robot.subsystems.arm;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants.arm.*;
import frc.robot.Constants;
import frc.robot.Constants.arm;

public class ArmIOSim implements ArmIO{
    private DCMotor neo2 = new DCMotor(12, 2.6, 105, 1.8, Units.rotationsPerMinuteToRadiansPerSecond(5676), 2);
    private DCMotor neo = new DCMotor(12, 2.6, 105, 1.8, Units.rotationsPerMinuteToRadiansPerSecond(5676), 1);

    private SparkMax angle = new SparkMax(ids.ANGLE_1_ID, MotorType.kBrushless);
    private SparkMaxSim angleSim = new SparkMaxSim(angle, neo2);

    private SparkMax extention = new SparkMax(ids.EXTENTION_1_ID, MotorType.kBrushless);
    private SparkMaxSim extentionSim = new SparkMaxSim(extention, neo2);
    
    private SparkMax wrist = new SparkMax(ids.WRIST_ID, MotorType.kBrushless);
    private SparkMaxSim wristSim = new SparkMaxSim(wrist, neo);

    private SingleJointedArmSim armSim = new SingleJointedArmSim(neo2, 100, 0.23459765, limits.MIN_EXTENTION, Units.degreesToRadians(limits.MIN_ANGLE), Units.degreesToRadians(limits.MAX_ANGLE), true, 0, 0, 0);

    private ProfiledPIDController anglePID = new ProfiledPIDController( simGains.ANGLE_KP,
                                                                        simGains.ANGLE_KI,
                                                                        simGains.ANGLE_KD,
                                                                        new Constraints(720, 720));
    private ArmFeedforward angleFF = new ArmFeedforward(simGains.ANGLE_KS,
                                                        simGains.ANGLE_KG,
                                                        simGains.ANGLE_KV);

    private ProfiledPIDController extentionPID = new ProfiledPIDController( simGains.EXTENTION_KP,
                                                                            simGains.EXTENTION_KI,
                                                                            simGains.EXTENTION_KD,
                                                                            new Constraints(100, 10));
    private ArmFeedforward extentionFF = new ArmFeedforward(simGains.EXTENTION_KS,
                                                            simGains.EXTENTION_KG,
                                                            simGains.EXTENTION_KV);

    private ProfiledPIDController wristPID = new ProfiledPIDController( simGains.WRIST_KP,
                                                                        simGains.WRIST_KI,
                                                                        simGains.WRIST_KD,
                                                                        new Constraints(800, 800));
    private ArmFeedforward wristFF = new ArmFeedforward(simGains.WRIST_KS,
                                                        simGains.WRIST_KG,
                                                        simGains.WRIST_KV);

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

    public ArmIOSim(){
    }
    @Override
    public void updateInputs(ArmIOInputs input){
        updateEncoders();
        //move
        
        angleSim.setAppliedOutput(anglePID.calculate(angleCurrent, angleSetpoint) + angleFF.calculate(angleCurrent, anglePID.getSetpoint().velocity));
        extentionSim.setAppliedOutput(extentionPID.calculate(extentionCurrent, extentionSetpoint) + extentionFF.calculate(extentionCurrent, extentionPID.getSetpoint().velocity));
        wristSim.setAppliedOutput(wristPID.calculate(wristCurrent, wristSetpoint) + wristFF.calculate(wristCurrent, wristPID.getSetpoint().velocity));
        
        input.armP1Pose = arm1pose;
        input.armP2Pose = arm2pose;
        input.armP3Pose = arm3pose;
        input.wristPose = wristpose;

        input.angleMotor1Temp = angle.getMotorTemperature();
        input.extentionMotor1Temp = extention.getMotorTemperature();
        input.wristMotorTemp = wrist.getMotorTemperature();

        input.angleAppliedOut = angle.getAppliedOutput();
        input.extentionAppliedOut = extention.getAppliedOutput();
        input.wristAppliedOut = wrist.getAppliedOutput();

        input.angleCurrentDrawAmps = angle.getOutputCurrent();
        input.extentionCurrentDrawAmps = extention.getOutputCurrent();
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
        angleCurrent = Units.radiansToRotations(armSim.getAngleRads())*simGains.ANGLE_FACTOR + arm.ANGLE_OFFSET;
        extentionCurrent = 0*simGains.EXTENTION_FACTOR + arm.EXTENTION_OFFSET;
        wristCurrent = 0*simGains.WRIST_FACTOR + arm.WRIST_OFFSET;
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