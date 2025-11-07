package frc.robot.subsystems.intake;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DigitalInput;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import static frc.robot.constants.IntakeConstants.*;
import static frc.utils.SparkUtil.*;

/**
 * intake IO implementation for Spark Max on real robot
 */
public class IntakeIOSpark implements IntakeIO {
    private final SparkMax motor = new SparkMax(MOTOR_ID, MotorType.kBrushless);
    private final SparkMaxConfig motorConfig = new SparkMaxConfig();
    private DigitalInput holdingSens = new DigitalInput(SENS_ID);
    private Debouncer holdingDebouncer = new Debouncer(0.06, Debouncer.DebounceType.kFalling);

    private double voltage = 0.0;

    public IntakeIOSpark() {
        // configure motors
        motorConfig
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(CURRENT_LIM)
                .voltageCompensation(12.0)
                .inverted(INVERTED);

        motorConfig.signals
                .appliedOutputPeriodMs(20)
                .busVoltagePeriodMs(20)
                .outputCurrentPeriodMs(20);

        configure();

    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        // Update control outputs based on control mode
        motor.setVoltage(voltage);

        // Update input values
        inputs.motorVoltage = motor.getBusVoltage() * motor.getAppliedOutput();
        inputs.motorCurrent = motor.getOutputCurrent();
        inputs.motorTemperature = motor.getMotorTemperature();
        inputs.motorVelocityRPM = motor.getEncoder().getVelocity();

        inputs.holding = holdingDebouncer.calculate(!holdingSens.get());
    }

    @Override
    public void setVoltage(double voltage) {
        this.voltage = voltage;
    }

    @Override
    public void setBrakeMode(boolean brake) {
        if (brake) {
            motorConfig.idleMode(IdleMode.kBrake);
        } else {
            motorConfig.idleMode(IdleMode.kCoast);
        }
        configure();
    }

    /**
     * configure motor with current config
     */
    private void configure() {
        tryUntilOk(
                motor,
                5,
                () -> motor.configure(
                        motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    }
}
