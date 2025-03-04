package frc.robot.subsystems.elevator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.utils.ElevatorFF;
import frc.utils.ProfiledPID;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import static frc.robot.constants.ElevatorConstants.*;
import static frc.utils.SparkUtil.*;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

public class ElevatorIOSpark implements ElevatorIO {

    private SparkMax motor1 = new SparkMax(MOTOR_1_ID, MotorType.kBrushless);
    private SparkMaxConfig motor1Config = new SparkMaxConfig();
    private SparkMax motor2 = new SparkMax(MOTOR_2_ID, MotorType.kBrushless);
    private SparkMaxConfig motor2Config = new SparkMaxConfig();
    private Encoder encoder = new Encoder(ENCODER_ID_A, ENCODER_ID_B);
    private RelativeEncoder motorEncoder = motor1.getEncoder();

    private double posOffset = 0.0;
    private double pos = 0.0;
    private double vel = 0.0;
    private double posSetpoint = 0.0;

    private double Builtinfactor = BUILTIN_POS_FACTOR * (BUILTIN_ENCODER_INVERT ? -1 : 1);

    private double voltsOut = 0.0;
    private boolean openloop = false;

    private boolean encoderFallback = true;

    private Alert divergenceAlert = new Alert("main encoder and builtin encoder values are divergent!", AlertType.kError);
    private Alert fallbackAlert = new Alert("using builtin encoder as fallback", AlertType.kWarning);

    private ProfiledPID pid = new ProfiledPID(POS_PID);
    private ElevatorFF ff = new ElevatorFF(POS_FF);

    private LoggedNetworkBoolean fallback = new LoggedNetworkBoolean("overrides/elevator encoder fallback", true);

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
    encoder.setSamplesToAverage(5);
    motorEncoder.setPosition(0);
    
    //pid.reset(encoder.getDistance() + posOffset);
    
    pid.reset(motorEncoder.getPosition()*Builtinfactor);
    }

    public void updateInputs(ElevatorIOInputs inputs) {
        encoderFallback = fallback.get();
        if(encoderFallback){
            vel = (motorEncoder.getVelocity() * Builtinfactor/60.0);
            pos = motorEncoder.getPosition() * Builtinfactor;
        } else {
            pos = encoder.getDistance() + posOffset;
            vel = encoder.getRate();
        }

        fallbackAlert.set(encoderFallback);
        divergenceAlert.set(MathUtil.isNear(motorEncoder.getPosition() * Builtinfactor, encoder.getDistance() + posOffset, ENCODER_DIVERGANCE_THRESH));

        double pidOut = pid.calculate(pos, posSetpoint);
        double ffOut = ff.calculate(pid.getSetpoint().velocity);
        Logger.recordOutput("Elevator/pidOut", pidOut);
        Logger.recordOutput("Elevator/ffOut", ffOut);
        Logger.recordOutput("Elevator/target", posSetpoint);
        Logger.recordOutput("Elevator/setpoint", pid.getSetpoint().position);
        if(!openloop){
            voltsOut = pidOut + ffOut;
        }
        motor1.setVoltage(voltsOut);

        inputs.elevatorPositionMeters = pos;
        inputs.elevatorVelocityMetersPerSec = vel;

        inputs.motor1CurrentAmps = motor1.getOutputCurrent();
        inputs.motor1Voltage = motor1.getBusVoltage()*motor1.getAppliedOutput();
        inputs.motor1TempC = motor1.getMotorTemperature();

        inputs.motor2CurrentAmps = motor2.getOutputCurrent();
        inputs.motor2Voltage = motor2.getBusVoltage()*motor2.getAppliedOutput();
        inputs.motor2TempC = motor2.getMotorTemperature();
    }
    
    public void setElevatorTargetLocation(double targetMeters) {
        posSetpoint = targetMeters;
        openloop = false;
    }
    
    public void moveElevatorOpenLoop(double voltage) {
        openloop = true;
        voltsOut = voltage;
        motor1.setVoltage(voltage);
    }
    
    public void setElevatorNeutralMode(boolean brake) {
        
        if(brake){
            motor1Config.idleMode(IdleMode.kBrake);
            motor2Config.idleMode(IdleMode.kBrake);
        } else {
            motor1Config.idleMode(IdleMode.kCoast);
            motor2Config.idleMode(IdleMode.kCoast);
        }
        configure();
    }
    
    public void resetElevatorPosition(double posMeters) {
        encoder.reset();
        posOffset = posMeters;
        motorEncoder.setPosition(posMeters/Builtinfactor);
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
                motor2Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    }
}
