// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public enum RobotMode{
    REAL,
    SIM,
    REPLAY
  }
  public static RobotMode SIM_MODE = RobotMode.SIM;
  public static RobotMode MODE = RobotBase.isReal() ? RobotMode.REAL : SIM_MODE;

  public static final double AUTO_TIME = 15;
  public static final double TELEOP_TIME = 135;//2:15
  public static final double ENDGAME_TIME = 20;//time remaining in teleop when endgame starts

  public static final Pose2d STARTING_POSE = new Pose2d(new Translation2d(2.3, 4), Rotation2d.fromDegrees(0));
  
  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag

  public static class Affector{

    public static final AffectorPosition HOLD_POSITION = new AffectorPosition(0.15, Units.degreesToRadians(90));
    public static final AffectorPosition STOW_POSITION = new AffectorPosition(0.0, Units.degreesToRadians(90));
    public static final AffectorPosition L1_POSITION = new AffectorPosition(0.08, -1.066);
    public static final AffectorPosition L2_POSITION = new AffectorPosition(0.157, 0.942);
    public static final AffectorPosition L3_POSITION = new AffectorPosition(0.592, 0.942);
    public static final AffectorPosition L4_POSITION = new AffectorPosition(1.6, 0.165);
    public static final AffectorPosition STATION_POSITION  = new AffectorPosition(0.48, -0.873);
  }

  public static class OperatorConstants
  {
    // Joystick Deadbands
    public static final double LEFT_DEADBAND  = 0.1;
    public static final double RIGHT_DEADBAND = 0.15;

    public static final double ANGLE_DEADBAND = 0.5;

    public static final double ELEVATOR_MAN_SENS = 0.05;
    
    //Curvature
    public static final double TRANSLATION_CURVE = 1.5;
    public static final double ROTATION_CURVE = 1.5;

    //usb port of driver controller, remember to assign controller to port in driverstation
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 1;
  }
  public static class drive {
    public static final boolean STARTING_FOD = true;
    public static final boolean STARTING_DIRECT_ANGLE = false;
  }
}
