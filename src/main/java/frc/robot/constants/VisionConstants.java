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

import java.nio.file.Path;
import java.util.function.Function;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.subsystems.vision.FilterStrategy;

public class VisionConstants {
  // AprilTag layout
  public static AprilTagFieldLayout APRILTAG_LAYOUT;
    //   

  // Camera names, must match names configured on coprocessor
  public static String[] CAMERA_NAMES = {
    "bl",
    "fl",
    "br"
  };

  // Robot to camera transforms
  public static Transform3d BL_ROBOT_TO_CAM =
        new Transform3d(
            0.288,
            0.22,
            -0.096,
        new Rotation3d(
            0,
            Units.degreesToRadians(-20),//-pitch
            -0.733
        )
  );
  public static Transform3d FL_ROBOT_TO_CAM =
        new Transform3d(
            0.292,
            -0.22,
            -0.1,
        new Rotation3d(
            0,
            Units.degreesToRadians(-20),//-pitch
            0.2189
        )
  );
  public static Transform3d BR_ROBOT_TO_CAM =
        new Transform3d(
            -Units.inchesToMeters(6),//forward
            -Units.inchesToMeters(11.5),//left 
            Units.inchesToMeters(10), //up
        new Rotation3d(
            Units.degreesToRadians(0),
            Units.degreesToRadians(-20),//-pitch
            Units.degreesToRadians(-85)//yaw
        )
  );
  

  // Basic filtering thresholds
  public static double MAX_AMBIGUITY = 0.75;
  public static double MAX_Z_ERROR = 0.75;

  // Standard deviation baselines, for 1 meter distance and 1 tag
  // (Adjusted automatically based on distance and # of tags)
  public static double LIN_STD_DEV_BASELINE = 0.2; // Meters
  public static double ANG_STD_DEV_BASELINE = 0.1; // Radians

  // Standard deviation multipliers for each camera
  // (Adjust to trust some cameras more than others)
  public static double[] CAM_STD_DEV_FACTORS =
      new double[] {
        1.0, // bl
        1.0, // fl
        1.0, // br
      };

  public static final FilterStrategy POSE_FILTER = FilterStrategy.MEAN;
  //SP and rate lim have problems with time
  //rate lim cant reset, will cause problems

}