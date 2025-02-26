package frc.robot.constants;

import edu.wpi.first.math.util.Units;

public class WristConstants {
    public static final double POS_OFFSET = 0.0;
    public static final double POS_FACTOR = Math.PI*2;

    public static final double POS_P = 0.0;
    public static final double POS_D = 0.0;
    public static final double POS_S = 0.0;
    public static final double POS_G = 0.0;
    public static final double POS_V = 0.0;
    public static final double POS_A = 0.0;
    public static final double POS_MAX_SPEED = 0.0;
    public static final double POS_MAX_ACCEL = 0.0;

    public static final int MOTOR_MAX_CURRENT = 40;

    public static final int ENCODER_ID = 3;
    public static final int MOTOR_ID = 3;

    public static final boolean MOTOR_INVERT = false;

    public static final double MAX_POS = Units.degreesToRadians(360);
    public static final double MIN_POS = Units.degreesToRadians(-360);
}
