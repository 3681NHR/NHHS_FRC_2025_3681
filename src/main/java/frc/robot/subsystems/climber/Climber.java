package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.constants.ClimberConstants.*;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class Climber extends SubsystemBase{

    private SparkMax motor = new SparkMax(MOTOR_ID, MotorType.kBrushless);

    private SparkMaxConfig config = new SparkMaxConfig();

    private double vout = 0.0;

    public Climber(){
         
    config
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(CURRENT_LIM)
        .voltageCompensation(12.0)
        .inverted(INVERT);
    config
        .signals
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);
    }

    @Override
    public void periodic(){
        motor.setVoltage(vout);
    }
    public void setVoltage(double vout){
        this.vout = vout;
    }
}
