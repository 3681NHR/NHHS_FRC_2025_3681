package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.*;

public class VisionEstimate {
    public Pose2d pose;
    public double timestampSeconds;
    public Matrix<N3, N1> visionMeasurementStdDevs;

    public VisionEstimate(Pose2d visionRobotPoseMeters, double timestampSeconds,
            Matrix<N3, N1> visionMeasurementStdDevs) {
        this.pose = visionRobotPoseMeters;
        this.timestampSeconds = timestampSeconds;
        this.visionMeasurementStdDevs = visionMeasurementStdDevs;
    }

    public VisionEstimate() {
        this.pose = new Pose2d();
        this.timestampSeconds = 0;
        this.visionMeasurementStdDevs = null;
    }
}