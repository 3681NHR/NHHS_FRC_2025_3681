package frc.robot.constants;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import frc.utils.PIDGains;

public class WristConstants {
    //encoder config
    public static final double POS_OFFSET = Units.degreesToRadians(20.37);
    public static final double POS_FACTOR = -2*Math.PI;

    //PIDs
    public static final PIDGains.ProfiledPID POS_PID = new PIDGains.ProfiledPID(1.5, 0, 0, 8, 17);
    public static final PIDGains.GravityFF POS_FF = new PIDGains.GravityFF(0.2, 0.356, 0.75, 0.25);
    public static final PIDGains.ProfiledPID POS_PID_SIM = new PIDGains.ProfiledPID(1.5, 0, 0, 8, 17);
    public static final PIDGains.GravityFF POS_FF_SIM = new PIDGains.GravityFF(0.2, 0.356, 0.75, 0.25);
    
    //IDs
    public static final int ENCODER_ID = 3;
    public static final int MOTOR_ID = 42;
    
    //motor config
    public static final int MOTOR_MAX_CURRENT = 20;
    public static final boolean MOTOR_INVERT = true;

    //software limits
    public static final double MAX_POS = Units.degreesToRadians(100);
    public static final double MIN_POS = Units.degreesToRadians(-70);

    //tolerances
    public static final double POS_TOLERANCE = 0.125;
    public static final double POS_NEAR_TOLERANCE = Units.degreesToRadians(30);

    //offset for ascope model
    public static final Translation3d WRIST_POS = new Translation3d(0.25, 0.18, 0.585);
    
    //sysid
    public static final Voltage VSTEP = Volts.of(1.5);
    public static final Velocity<VoltageUnit> VRAMP = Volts.of(.5).per(Second);
    public static final Time TIMEOUT = Seconds.of(7);
}
