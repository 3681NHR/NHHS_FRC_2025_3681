package frc.robot.subsystems.wrist;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.utils.SparkUtil;
import frc.utils.ExtraMath.Derrivitive;

import static frc.robot.constants.WristConstants.*;

import org.littletonrobotics.junction.Logger;

public class WristIOSpark implements WristIO{

    private double pos;
    private double vel;
    private double posSet;
    private DutyCycleEncoder encoder = new DutyCycleEncoder(ENCODER_ID);

    private ArmFeedforward ff = new ArmFeedforward(POS_S, POS_G, POS_V, POS_A);

    private ProfiledPIDController pid = new ProfiledPIDController(POS_P, 0, POS_D, new Constraints(POS_MAX_SPEED, POS_MAX_ACCEL));

    private SparkMax motor = new SparkMax(MOTOR_ID, MotorType.kBrushless);
    private SparkMaxConfig config = new SparkMaxConfig();

    private Derrivitive velDeriv = new Derrivitive();

    public WristIOSpark(){
        config
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(MOTOR_MAX_CURRENT)
            .voltageCompensation(12.0)
            .inverted(MOTOR_INVERT);
        config
            .signals
            .appliedOutputPeriodMs(20)
            .busVoltagePeriodMs(20)
            .outputCurrentPeriodMs(20);

        SparkUtil.tryUntilOk(motor, 
        5, 
        () -> motor.configure(
            config, 
            ResetMode.kResetSafeParameters, 
            PersistMode.kPersistParameters
        ));
    }

    @Override
    public void setPos(double pos){
        posSet = pos;
    }

    @Override
    public void updateInputs(WristIOInputs in){
        pos = (encoder.get()*POS_FACTOR) + POS_OFFSET;
        vel = velDeriv.calculate(pos, 0.02);

        double pidOut = pid.calculate(pos, posSet);
        double ffOut = ff.calculate(pos, pid.getSetpoint().velocity);

        Logger.recordOutput("Wrist/PID goal", posSet);
        Logger.recordOutput("Wrist/PID set", pid.getSetpoint().position);
        Logger.recordOutput("Wrist/PID out", pidOut);
        Logger.recordOutput("Wrist/FF out", ffOut);

        motor.setVoltage(pidOut + ffOut);

        in.motorCurrentAmps = motor.getOutputCurrent();
        in.motoroutVolts = motor.getBusVoltage()*motor.getAppliedOutput();
        in.motorTemp = motor.getMotorTemperature();

        in.posRad = pos;
        in.velRadPerSec = vel;
    }
}
