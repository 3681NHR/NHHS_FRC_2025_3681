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
    private final SimpleMotorFeedforward ff = new SimpleMotorFeedforward(S, V);
    private final PIDController pidController = new PIDController(P, 0, D);
    
    private double targetVoltage = 0.0;
    private boolean closedLoop = false;
    private double targetVelocity = 0.0;
    private double appliedVoltage = 0.0;
    
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
        if (closedLoop) {
            // Use PID + Feedforward for velocity control
            double currentVelocity = encoder.getVelocity();
            double pidOutput = pidController.calculate(currentVelocity, targetVelocity);
            double ffOutput = ff.calculate(targetVelocity);
            appliedVoltage = pidOutput + ffOutput;
            
            // Log control values
            Logger.recordOutput("Intake/PIDOutput", pidOutput);
            Logger.recordOutput("Intake/FFOutput", ffOutput);
            Logger.recordOutput("Intake/TargetVelocity", targetVelocity);
            
            motor.setVoltage(appliedVoltage);
        } else {
            // Direct voltage control
            appliedVoltage = targetVoltage;
            motor.setVoltage(targetVoltage);
        }
        
        // Update input values
        inputs.motorVoltage = motor.getBusVoltage() * motor.getAppliedOutput();
        inputs.motorCurrent = motor.getOutputCurrent();
        inputs.motorTemperature = motor.getMotorTemperature();
        inputs.motorVelocityRPM = encoder.getVelocity();
        inputs.motorPositionRotations = encoder.getPosition();
    }
    
    @Override
    public void moveOpenLoop(double voltage) {
        closedLoop = false;
        targetVoltage = voltage;
    }
    
    @Override
    public void setVoltage(double voltage) {
        closedLoop = false;
        targetVoltage = voltage;
    }
    
    public void setVelocity(double velocityRPM) {
        closedLoop = true;
        targetVelocity = velocityRPM;
        pidController.setSetpoint(velocityRPM);
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
    
    @Override
    public void resetposition(double posMeters) {
        encoder.setPosition(posMeters);
    }
    
    private void configure() {
        tryUntilOk(
            motor,
            5,
            () -> motor.configure(
                motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    }
}
