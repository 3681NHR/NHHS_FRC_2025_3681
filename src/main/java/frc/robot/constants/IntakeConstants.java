package frc.robot.constants;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class IntakeConstants {
    public static final int MOTOR_ID = 51;
    public static final int CURRENT_LIM = 20;
    public static final boolean INVERTED = false;
    public static final IdleMode IDLE_MODE = IdleMode.kBrake;
    public static final double MOTOR_RUNNING_THRESHOLD = 1.0;
}