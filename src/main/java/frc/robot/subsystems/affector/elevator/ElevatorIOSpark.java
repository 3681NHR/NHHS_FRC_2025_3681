package frc.robot.subsystems.affector.elevator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import static frc.robot.constants.ElevatorConstants.*;
import static frc.utils.SparkUtil.*;

/**
 * elevator IO that interfaces with Spark Max motor controllers
 */
public class ElevatorIOSpark implements ElevatorIO {

    private SparkMax motor1 = new SparkMax(MOTOR_1_ID, MotorType.kBrushless);
    private SparkMaxConfig motor1Config = new SparkMaxConfig();
    private SparkMax motor2 = new SparkMax(MOTOR_2_ID, MotorType.kBrushless);
    private SparkMaxConfig motor2Config = new SparkMaxConfig();
    private RelativeEncoder motorEncoder = motor1.getEncoder();

    private double pos = 0.0;
    private double vel = 0.0;

    private double builtinfactor = BUILTIN_POS_FACTOR * (BUILTIN_ENCODER_INVERT ? -1 : 1);

    private double voltsOut = 0.0;

    public ElevatorIOSpark() {
        // configure motor 1
        motor1Config
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(CURRENT_LIM)
                .voltageCompensation(12.0)
                .inverted(MOTOR_INVERT);
        motor1Config.signals
                .appliedOutputPeriodMs(20)
                .busVoltagePeriodMs(20)
                .outputCurrentPeriodMs(20);
        // set motor 2 to follow motor 1 with the same config
        motor2Config.apply(motor1Config).follow(motor1, true);

        configure();

        // reset sensor on boot
        motorEncoder.setPosition(0);
    }

    public void updateInputs(ElevatorIOInputs inputs) {
        motor1.setVoltage(voltsOut);

        // convert to m/s
        vel = (motorEncoder.getVelocity() * builtinfactor / 60.0);
        // convert to meters
        pos = motorEncoder.getPosition() * builtinfactor;

        inputs.pos = pos;
        inputs.vel = vel;

        inputs.motor1CurrentAmps = motor1.getOutputCurrent();
        inputs.motor1Voltage = motor1.getBusVoltage() * motor1.getAppliedOutput();
        inputs.motor1TempC = motor1.getMotorTemperature();

        inputs.motor2CurrentAmps = motor2.getOutputCurrent();
        inputs.motor2Voltage = motor2.getBusVoltage() * motor2.getAppliedOutput();
        inputs.motor2TempC = motor2.getMotorTemperature();
    }

    public void setVoltage(double voltage) {
        voltsOut = voltage;
        // motor 2 will follow motor 1 so it doesnt need to be set
        motor1.setVoltage(voltage);
    }

    public void setBrake(boolean brake) {
        // configure both motors' brake mode
        if (brake) {
            motor1Config.idleMode(IdleMode.kBrake);
            motor2Config.idleMode(IdleMode.kBrake);
        } else {
            motor1Config.idleMode(IdleMode.kCoast);
            motor2Config.idleMode(IdleMode.kCoast);
        }
        configure();
    }

    public void resetPos(double posMeters) {
        motorEncoder.setPosition(posMeters / builtinfactor);
    }

    /**
     * configure both motors with the current configuration
     */
    private void configure() {

        tryUntilOk(
                motor1,
                5,
                () -> motor1.configure(
                        motor1Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
        tryUntilOk(
                motor2,
                5,
                () -> motor2.configure(
                        motor2Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    }
}
