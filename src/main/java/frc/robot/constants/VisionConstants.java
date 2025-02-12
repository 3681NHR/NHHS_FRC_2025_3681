// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.constants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.vision.FilterStrategy;

public class VisionConstants {
  // AprilTag layout
  public static AprilTagFieldLayout APRILTAG_LAYOUT =
      AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  // Camera names, must match names configured on coprocessor
  public static String CAMERA_0_NAME = "front";

  // Robot to camera transforms
  // (Not used by Limelight, configure in web UI instead)
  public static Transform3d CAMERA_0_ROBOT_TO_CAM =
        new Transform3d(
            Units.inchesToMeters(14), 
            Units.inchesToMeters(0), 
            Units.inchesToMeters(4), 
        new Rotation3d(
            Units.degreesToRadians(0),
            Units.degreesToRadians(20),
            Units.degreesToRadians(45)
        )
    );

  // Basic filtering thresholds
  public static double MAX_AMBIGUITY = 0.3;
  public static double MAX_Z_ERROR = 0.75;

  // Standard deviation baselines, for 1 meter distance and 1 tag
  // (Adjusted automatically based on distance and # of tags)
  public static double LIN_STD_DEV_BASELINE = 0.02; // Meters
  public static double ANG_STD_DEV_BASELINE = 0.06; // Radians

  // Standard deviation multipliers for each camera
  // (Adjust to trust some cameras more than others)
  public static double[] CAM_STD_DEV_FACTORS =
      new double[] {
        1.0, // front
      };

  public static final FilterStrategy POSE_FILTER = FilterStrategy.RATE_LIM;
}