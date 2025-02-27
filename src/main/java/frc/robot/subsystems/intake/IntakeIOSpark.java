package frc.robot.subsystems.intake;

import static frc.robot.constants.IntakeConstants.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import org.littletonrobotics.junction.Logger;

import static frc.utils.SparkUtil.*;

public class IntakeIOSpark implements IntakeIO {
    private final SparkMax motor = new SparkMax(MOTOR_ID, MotorType.kBrushless);
    private final SparkMaxConfig motorConfig = new SparkMaxConfig();
    private final RelativeEncoder encoder = motor.getEncoder();
    
    private double voltage = 0.0;
    
    public IntakeIOSpark() {
        motorConfig
            .idleMode(IDLE_MODE)
            .smartCurrentLimit(CURRENT_LIM)
            .voltageCompensation(12.0)
            .inverted(INVERTED);
            
        motorConfig
            .signals
            .appliedOutputPeriodMs(20)
            .busVoltagePeriodMs(20)
            .outputCurrentPeriodMs(20);
            
        configure();
        
        // Reset encoder position
        encoder.setPosition(0);
    }
    
    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        // Update control outputs based on control mode
        motor.setVoltage(voltage);
        
        
        // Update input values
        inputs.motorVoltage = motor.getBusVoltage() * motor.getAppliedOutput();
        inputs.motorCurrent = motor.getOutputCurrent();
        inputs.motorTemperature = motor.getMotorTemperature();
        inputs.motorVelocityRPM = encoder.getVelocity();
        inputs.motorPositionRotations = encoder.getPosition();
    }
    
    @Override
    public void setVoltage(double voltage) {
        this.voltage = voltage;
    }

    @Override
    public void setBrakeMode(boolean brake) {
        setNeutralMode(brake);
    }
    
    @Override
    public void setNeutralMode(boolean brake) {
        if (brake) {
            motorConfig.idleMode(IdleMode.kBrake);
        } else {
            motorConfig.idleMode(IdleMode.kCoast);
        }
        configure();
    }
    
    
    private void configure() {
        tryUntilOk(
            motor,
            5,
            () -> motor.configure(
                motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    }
}
