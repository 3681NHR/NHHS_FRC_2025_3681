package frc.robot.subsystems.elevator;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.Encoder;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import static frc.robot.constants.ElevatorConstants.*;
import static frc.utils.SparkUtil.*;

public class ElevatorIOSpark implements ElevatorIO {

    private SparkMax motor1 = new SparkMax(0, MotorType.kBrushless);
    private Encoder encoder = new Encoder(ENCODER_ID_A, ENCODER_ID_B);

    public ElevatorIOSpark(){
        
    var motorConfig = new SparkMaxConfig();
    motorConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(CURRENT_LIM)
        .voltageCompensation(12.0);
    motorConfig
        .signals
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);
    tryUntilOk(
        motor1,
        5,
        () ->
            motor1.configure(
                motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    encoder.reset();
    encoder.setDistancePerPulse(POS_FACTOR);
    }

    public void updateInputs(ElevatorIOInputs inputs) {

    }
    
    public void setTargetLocation(double targetMeters) {

    }
    
    public void moveOpenLoop(double voltage) {

    }
    
    public void setNutralMode(boolean brake) {

    }
    
    public void resetposition(double posMeters) {

    }
}
