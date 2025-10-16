package frc.robot.constants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;

public class VisionConstants {
    // AprilTag layout on field, loaded from json file in robotcontainer
    public static AprilTagFieldLayout APRILTAG_LAYOUT;

    // Camera names, must match names configured on coprocessor
    public static String[] CAMERA_NAMES = {
            "bl",
            "fl",
            "br"
    };

    // Robot to camera transforms
    public static Transform3d BL_ROBOT_TO_CAM = new Transform3d(
            -0.2135,
            0.287,
            0,
            new Rotation3d(
                    0,
                    Units.degreesToRadians(-20), // -pitch
                    -2.289 + 3.1415926535897932384626433832));
    public static Transform3d FL_ROBOT_TO_CAM = new Transform3d(
            0.235,
            0.301,
            0,
            new Rotation3d(
                    0,
                    Units.degreesToRadians(-20), // -pitch
                    -1.333 + 3.1415926535897932384626433832));
    public static Transform3d BR_ROBOT_TO_CAM = new Transform3d(
            -0.113,
            -0.3208,
            0,
            new Rotation3d(
                    Units.degreesToRadians(0),
                    Units.degreesToRadians(-20), // -pitch
                    1.6315 + 3.1415926535897932384626433832));

    // Basic filtering thresholds
    public static double MAX_AMBIGUITY = 0.75;
    public static double MAX_Z_ERROR = 0.75;

    // Standard deviation baselines, for 1 meter distance and 1 tag
    // (Adjusted automatically based on distance and # of tags)
    public static double LIN_STD_DEV_BASELINE = 0.2; // Meters
    public static double ANG_STD_DEV_BASELINE = 0.1; // Radians

    // Standard deviation multipliers for each camera
    // (Adjust to trust some cameras more than others)
    public static double[] CAM_STD_DEV_FACTORS = new double[] {
            1.0, // bl
            1.0, // fl
            0.8, // br
    };
}