package frc.robot.constants;

import edu.wpi.first.math.util.Units;

public final class ElevatorConstants {
    
    public static final double HOME_VOLTAGE = -0.5;
    public static final double HOME_MIN_VEL = 0.1;
    public static final double HOME_STOP_TIME = 0.1;
    public static final double HOME_POS = 0;

    public static final double MIN_POS = 0.0;
    public static final double MAX_POS = 1.75;

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

    public static final double POS_P = 5;
    public static final double POS_D = 0.0;

    public static final double POS_S = 0.2;
    public static final double POS_V = 4;
    public static final double POS_A = 0.0;
    public static final double POS_G = 0.2;
    
    public static final double SIM_POS_P = 2;
    public static final double SIM_POS_D = 0.0;

    public static final double SIM_POS_S = .05;
    public static final double SIM_POS_V = 11;
    public static final double SIM_POS_A = 0.0;
    public static final double SIM_POS_G = 0.2222222;

    public static final double POS_MAX_SPEED = 1.75;
    public static final double POS_MAX_ACCEL = POS_MAX_SPEED*15;

    public static final double GEARING = 12;
    public static final double DRUM_RAD = Units.inchesToMeters(1);
    public static final double MASS = Units.lbsToKilograms(10);
}
