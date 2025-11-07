package frc.robot.subsystems.vision;

public enum FilterStrategy {
    RAW,
    RATE_LIM,
    SINGLE_POLE_IIR,
    MEAN,
    MEDIAN;
}