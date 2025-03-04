package frc.robot.constants;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import frc.utils.PIDGains;

public final class ElevatorConstants {
    
    public static final double HOME_VOLTAGE = -0.5;
    public static final double HOME_MIN_VEL = 0.1;
    public static final double HOME_STOP_TIME = 0.1;
    public static final double HOME_POS = 0;

    public static final double MIN_POS = 0.0;
    public static final double MAX_POS = 1.6;

    public static final double DIST_PER_PULSE = Math.PI*2 / 2048;
    public static final double BUILTIN_POS_FACTOR = 0.0266670930055;
    public static final double POS_FACTOR = DIST_PER_PULSE * 1;

    public static final double ENCODER_DIVERGANCE_THRESH = 1;
    
    public static final int MOTOR_1_ID = 40;
    public static final int MOTOR_2_ID = 41;

    public static final int ENCODER_ID_A = 1;
    public static final int ENCODER_ID_B = 0;

    public static final boolean MOTOR_INVERT = false;
    public static final boolean BUILTIN_ENCODER_INVERT = false;
    public static final boolean ENCODER_INVERT = false;

    public static final int CURRENT_LIM = 30;

    public static final PIDGains.ProfiledPID POS_PID = new PIDGains.ProfiledPID(0, 0, 0, 1.75, 1.75*15);
    public static final PIDGains.GravityFF POS_FF = new PIDGains.GravityFF(0.19349, 0.35424, 4.5227, 0.3535);

    public static final PIDGains.ProfiledPID POS_PID_SIM = new PIDGains.ProfiledPID(0, 0, 0, 1.75, 1.75*5);
    public static final PIDGains.GravityFF POS_FF_SIM = new PIDGains.GravityFF(0.2, 0.2, 4, 0.0);

    public static final double POS_TOLERANCE = Units.inchesToMeters(2.5);
    
    public static final Voltage VSTEP = Volts.of(5);
    public static final Velocity<VoltageUnit> VRAMP = Volts.of(.5).per(Second);
    public static final Time TIMEOUT = Seconds.of(5);
}
