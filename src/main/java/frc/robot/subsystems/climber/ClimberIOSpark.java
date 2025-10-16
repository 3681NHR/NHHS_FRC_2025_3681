package frc.robot.subsystems.climber;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.utils.SparkUtil;

import com.revrobotics.spark.config.SparkMaxConfig;

import static frc.robot.constants.ClimberConstants.*;

/**
 * climber IO implementation for Spark Max on real robot
 */
public class ClimberIOSpark implements ClimberIO {

    private SparkMax motor = new SparkMax(MOTOR_ID, MotorType.kBrushless);

    private SparkMaxConfig config = new SparkMaxConfig();

    public ClimberIOSpark() {
        // configure motor
        config
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(CURRENT_LIM)
                .voltageCompensation(12.0)
                .inverted(INVERTED);
        config.signals
                .appliedOutputPeriodMs(20)
                .busVoltagePeriodMs(20)
                .outputCurrentPeriodMs(20);

        SparkUtil.tryUntilOk(motor, 5,
                () -> motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    }

    public void setVoltage(double v) {
        motor.setVoltage(v);
    }

    public void updateInput(ClimberIOInputs in) {
        in.appliedVolts = motor.getAppliedOutput() * motor.getBusVoltage();
        in.currentDraw = motor.getOutputCurrent();
        in.motorTemp = motor.getMotorTemperature();
    }
}
