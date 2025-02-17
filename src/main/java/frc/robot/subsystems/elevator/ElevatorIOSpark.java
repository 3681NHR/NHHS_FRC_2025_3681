package frc.robot.subsystems.elevator;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.Encoder;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import static frc.robot.constants.ElevatorConstants.*;
import static frc.utils.SparkUtil.*;

import org.littletonrobotics.junction.Logger;

public class ElevatorIOSpark implements ElevatorIO {

    private SparkMax motor1 = new SparkMax(MOTOR_1_ID, MotorType.kBrushless);
    private SparkMaxConfig motor1Config = new SparkMaxConfig();
    private SparkMax motor2 = new SparkMax(MOTOR_2_ID, MotorType.kBrushless);
    private SparkMaxConfig motor2Config = new SparkMaxConfig();
    private Encoder encoder = new Encoder(ENCODER_ID_A, ENCODER_ID_B);

    private double posOffset = 0.0;
    private double pos = 0.0;
    private double vel = 0.0;
    private double posSetpoint = 0.0;

    private double voltsOut = 0.0;
    private boolean openloop = false;

    private ProfiledPIDController pid = new ProfiledPIDController(
        POS_P,
        0,
        POS_D,
        new Constraints(
            POS_MAX_SPEED,
            POS_MAX_ACCEL
        )
    );
    private ElevatorFeedforward ff = new ElevatorFeedforward(
        POS_S,
        POS_G,
        POS_V,
        POS_A
    );

    public ElevatorIOSpark(){
        
    motor1Config
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(CURRENT_LIM)
        .voltageCompensation(12.0)
        .inverted(MOTOR_INVERT);
    motor1Config
        .signals
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);

    motor2Config.apply(motor1Config).follow(motor1, true);

    configure();

    encoder.reset();
    encoder.setDistancePerPulse(POS_FACTOR);
    encoder.setReverseDirection(ENCODER_INVERT);
    }

    public void updateInputs(ElevatorIOInputs inputs) {
        vel = ((encoder.getDistance() + posOffset)-pos)/0.02;
        pos = encoder.getDistance() + posOffset;

        double pidOut = pid.calculate(pos, posSetpoint);
        double ffOut = ff.calculate(pid.getSetpoint().velocity);
        Logger.recordOutput("elevator/pidOut", pidOut);
        Logger.recordOutput("elevator/ffOut", ffOut);
        Logger.recordOutput("elevator/target", posSetpoint);
        Logger.recordOutput("elevator/setpoint", pid.getSetpoint().position);
        if(!openloop){
            voltsOut = pidOut + ffOut;
        }
        motor1.setVoltage(voltsOut);

        inputs.positionMeters = pos;
        inputs.velocityMetersPerSec = vel;

        inputs.motor1CurrentAmps = motor1.getOutputCurrent();
        inputs.motor1Voltage = motor1.getBusVoltage()*motor1.getAppliedOutput();
        inputs.motor1TempC = motor1.getMotorTemperature();

        inputs.motor2CurrentAmps = motor2.getOutputCurrent();
        inputs.motor2Voltage = motor2.getBusVoltage()*motor2.getAppliedOutput();
        inputs.motor2TempC = motor2.getMotorTemperature();
    }
    
    public void setTargetLocation(double targetMeters) {
        posSetpoint = targetMeters;
        openloop = false;
    }
    
    public void moveOpenLoop(double voltage) {
        openloop = true;
        voltsOut = voltage;
        motor1.setVoltage(voltage);
    }
    
    public void setNeutralMode(boolean brake) {
        if(brake){
            motor1Config.idleMode(IdleMode.kBrake);
            motor2Config.idleMode(IdleMode.kBrake);
        } else {
            motor1Config.idleMode(IdleMode.kCoast);
            motor2Config.idleMode(IdleMode.kCoast);
        }
        configure();
    }
    
    public void resetposition(double posMeters) {
        encoder.reset();
        posOffset = posMeters;
    }

    private void configure(){
        
    tryUntilOk(
        motor1,
        5,
        () ->
            motor1.configure(
                motor1Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    tryUntilOk(
        motor2,
        5,
        () ->
        motor2.configure(
            motor1Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    }
                
}
