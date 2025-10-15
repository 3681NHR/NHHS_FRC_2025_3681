package frc.robot.constants;

import edu.wpi.first.math.geometry.Translation3d;

public class IntakeConstants {
    //IDs
    public static final int MOTOR_ID = 51;
    public static final int SENS_ID = 2;
    
    //vout presets
    public static final double SPEED_INTAKE = 4;
    public static final double SPEED_SCORE = 10;

    //motor config
    public static final double MOTOR_RUNNING_THRESHOLD = 1.0;
    public static final int CURRENT_LIM = 20;
    public static final boolean INVERTED = true;
    
    //offset for ascope model
    public static final Translation3d WRIST_POS = new Translation3d(0.25, -0.25, 0.585);
    public static final double pivotToCoral = 0.15;
}