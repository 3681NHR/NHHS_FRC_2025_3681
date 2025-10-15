package frc.robot.constants;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import frc.utils.PIDGains;

public final class ElevatorConstants {
    
    //homing constants
    public static final double HOME_VOLTAGE = -0.75;
    public static final double HOME_MIN_VEL = 0.01;
    public static final double HOME_STOP_TIME = 0.25;//seconds
    public static final double HOME_POS = 0;

    //software limits
    public static final double MIN_POS = 0.0;
    public static final double MAX_POS = 1.6;

    //encoder factors
    public static final double DIST_PER_PULSE = Math.PI*2 / 2048;
    public static final double BUILTIN_POS_FACTOR = 0.0266670930055;
    public static final double POS_FACTOR = DIST_PER_PULSE * 1;//FIXME
    
    //CAN IDs
    public static final int MOTOR_1_ID = 40;
    public static final int MOTOR_2_ID = 41;

    //encoder ports(DIO)
    public static final int ENCODER_ID_A = 1;
    public static final int ENCODER_ID_B = 0;

    //motor config
    public static final int CURRENT_LIM = 35;
    public static final boolean MOTOR_INVERT = false;
    public static final boolean BUILTIN_ENCODER_INVERT = false;

    //PIDs
    public static final PIDGains.ProfiledPID POS_PID = new PIDGains.ProfiledPID(10, 0, 3, 2.5, 5);
    public static final PIDGains.GravityFF POS_FF = new PIDGains.GravityFF(0.3, 0.55, 5.0, 0.6);
    public static final PIDGains.ProfiledPID POS_PID_SIM = new PIDGains.ProfiledPID(10, 0, 3, 2.5, 5);
    public static final PIDGains.GravityFF POS_FF_SIM = new PIDGains.GravityFF(0, 0, 4, 0.6);
    //tolerance
    public static final double POS_TOLERANCE = 0.05;
    public static final double NEAR_POS_TOLERANCE = 0.25;
    
    //sysid
    public static final Voltage VSTEP = Volts.of(5);
    public static final Velocity<VoltageUnit> VRAMP = Volts.of(.5).per(Second);
    public static final Time TIMEOUT = Seconds.of(5);
}
