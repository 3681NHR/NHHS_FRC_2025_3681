package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;

public interface CameraIO {
    @AutoLog
    public static class CameraIOInputs {
        public boolean connected = false;
        public PoseObservation[] poseObservations = new PoseObservation[0];
        public int[] tagIds = new int[0];
        public TargetObservation latestTargetObservation = new TargetObservation(new Rotation2d(), new Rotation2d(),
                -1);
        public TargetObservation[] targets = new TargetObservation[0];
    }

    /** Represents the angle to a simple target, not used for pose estimation. */
    public static record TargetObservation(Rotation2d tx, Rotation2d ty, int ID) {
    }

    /** Represents a robot pose sample used for pose estimation. */
    public static record PoseObservation(
            double timestamp,
            Pose3d pose,
            double ambiguity,
            int tagCount,
            double averageTagDistance) {
    }

    public default void updateInputs(CameraIOInputs inputs) {
    }

    public default String getName() {
        return null;
    }

    public default Transform3d getRobotToCamera() {
        return new Transform3d();
    }
}
