package frc.robot.subsystems.elevator;

import static frc.robot.constants.ElevatorConstants.*;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import frc.robot.subsystems.elevator.ElevatorIO.ElevatorIOInputs;
import frc.utils.BatteryVoltageSim;

public class ElevatorIOSim implements ElevatorIO{

    
        private ElevatorSim sim = new ElevatorSim(
            DCMotor.getNEO(2),
            GEARING,
            MASS,
            DRUM_RAD,
            MIN_POS,
            MAX_POS,
            true,
            1.0,
            0.01,
            0.0
        );
    
        
        private Encoder encoder = new Encoder(ENCODER_ID_A, ENCODER_ID_B);
        private EncoderSim encoder_sim = new EncoderSim(encoder);

        private LinearFilter posFilter = LinearFilter.movingAverage(5);
        
        private double posOffset = 0.0;
        private double pos = 0.0;
        private double lastPos = 0.0;
        private double vel = 0.0;
        private double posSetpoint = 0.0;
    
        private double voltsOut = 0.0;
        private boolean openloop = false;
    
        private ProfiledPIDController pid = new ProfiledPIDController(
            SIM_POS_P,
            0,
            SIM_POS_D,
            new Constraints(
            POS_MAX_SPEED,
            POS_MAX_ACCEL
        )
    );
    private ElevatorFeedforward ff = new ElevatorFeedforward(
        SIM_POS_S,
        SIM_POS_G,
        SIM_POS_V,
        SIM_POS_A
    );

    public ElevatorIOSim(){
        BatteryVoltageSim.getInstance().addCurrentSource(sim::getCurrentDrawAmps);

        
        encoder.reset();
        encoder.setDistancePerPulse(POS_FACTOR);
        encoder.setReverseDirection(ENCODER_INVERT);

        pid.reset(encoder.getDistance() + posOffset);
    }
    
    
    public void updateInputs(ElevatorIOInputs inputs) {
        encoder_sim.setDistance(sim.getPositionMeters());
        pos = posFilter.calculate(encoder.getDistance() + posOffset);
        vel = (pos-lastPos)/0.02;
        lastPos = pos;

        posSetpoint = MathUtil.clamp(posSetpoint, MIN_POS, MAX_POS);

        double pidOut = pid.calculate(pos, posSetpoint);
        double ffOut = ff.calculate(pid.getSetpoint().velocity);
        Logger.recordOutput("elevator/pidOut", pidOut);
        Logger.recordOutput("elevator/ffOut", ffOut);
        Logger.recordOutput("elevator/target", posSetpoint);
        Logger.recordOutput("elevator/setpoint", pid.getSetpoint().position);
        if(!openloop){
            voltsOut = pidOut + ffOut;
        }
        sim.setInputVoltage(voltsOut);

        inputs.positionMeters = pos;
        inputs.velocityMetersPerSec = vel;

        inputs.motor1CurrentAmps = sim.getCurrentDrawAmps()/2;
        inputs.motor1Voltage = voltsOut/2;
        inputs.motor1TempC = -1;

        inputs.motor2CurrentAmps = inputs.motor1CurrentAmps;
        inputs.motor2Voltage = inputs.motor1Voltage;
        inputs.motor2TempC = inputs.motor1TempC;

        sim.update(0.02);
    }
    
    public void setTargetLocation(double targetMeters) {
        posSetpoint = targetMeters;
        openloop = false;
    }
    
    public void moveOpenLoop(double voltage) {
        openloop = true;
        voltsOut = voltage;
        sim.setInputVoltage(voltsOut);
    }
    
    public void setNeutralMode(boolean brake) {
        //not used with sim
    }
    
    public void resetposition(double posMeters) {
        //TODO reset
        posOffset = posMeters;
    }

}
