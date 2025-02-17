package frc.robot.constants;

public final class ElevatorConstants {
    
    public static final double HOME_VOLTAGE = -1.5;
    public static final double HOME_MIN_VEL = 0.01;
    public static final double HOME_STOP_TIME = 0.1;
    public static final double HOME_POS = 0;

    public static final double MIN_POS = 0.0;
    public static final double MAX_POS = 2;

    public static final double DIST_PER_PULSE = Math.PI*2 / 2048;
    public static final double POS_FACTOR = DIST_PER_PULSE * 1;//TODO
    public static final double VEL_FACTOR = POS_FACTOR;
    
    public static final int MOTOR_1_ID = 41;
    public static final int MOTOR_2_ID = 42;

    public static final int ENCODER_ID_A = 0;
    public static final int ENCODER_ID_B = 1;

    public static final boolean MOTOR_INVERT = false;
    public static final boolean ENCODER_INVERT = false;

    public static final int CURRENT_LIM = 40;

    public static final double POS_P = 0.0;
    public static final double POS_D = 0.0;

    public static final double POS_S = 0.0;
    public static final double POS_V = 0.0;
    public static final double POS_A = 0.0;
    public static final double POS_G = 0.0;

    public static final double POS_MAX_SPEED = 0.0;
    public static final double POS_MAX_ACCEL = 0.0;
}
