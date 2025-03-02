package frc.robot.constants;

import edu.wpi.first.math.util.Units;

public class WristConstants {
    public static final double POS_OFFSET = Units.degreesToRadians(20.37);
    public static final double POS_FACTOR = -2*Math.PI;

    public static final double POS_P = 0.0;
    public static final double POS_D = 0.0;
    public static final double POS_S = 0.0;
    public static final double POS_G = 0.5;
    public static final double POS_V = 0.3;
    public static final double POS_A = 0.0;
    public static final double POS_MAX_SPEED = 1;
    public static final double POS_MAX_ACCEL = POS_MAX_SPEED*3;

    public static final int MOTOR_MAX_CURRENT = 30;

    public static final int ENCODER_ID = 3;
    public static final int MOTOR_ID = 42;

    public static final boolean MOTOR_INVERT = true;

    public static final double MAX_POS = Units.degreesToRadians(100);
    public static final double MIN_POS = Units.degreesToRadians(-90);
}
