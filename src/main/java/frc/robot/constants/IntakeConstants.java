package frc.robot.constants;

import edu.wpi.first.math.geometry.Translation3d;

public class IntakeConstants {
    public static final int MOTOR_ID = 51;
    public static final int SENS_ID = 4;
    
    public static final double SPEED = 5;
    public static final double MOTOR_RUNNING_THRESHOLD = 1.0;
    
    public static final int CURRENT_LIM = 20;

    public static final boolean INVERTED = true;
    
    public static final Translation3d WRIST_POS = new Translation3d(0.25, -0.25, 0.585);
    public static final double pivotToCoral = 0.15;
    public static final double coralOut = -0.1;
}