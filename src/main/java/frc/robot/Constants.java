// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final double AUTO_TIME = 15;
  public static final double TELEOP_TIME = 135;//2:15

  public static class OperatorConstants
  {
    // Joystick Deadbands
    public static final double LEFT_X_DEADBAND  = 0.05;
    public static final double LEFT_Y_DEADBAND  = 0.05;
    public static final double RIGHT_X_DEADBAND = 0.05;
    public static final double RIGHT_Y_DEADBAND = 0.05;
    
    //Curvature (rotation curve only applies when using anguler velocity)
    public static final double TRANSLATION_CURVE = 2;
    public static final double ROTATION_CURVE = 2;
    
    public static final Rotation2d DPAD_UP_ANGLE    =  Rotation2d.fromDegrees(0);//rotation setpoint for dpad up button
    public static final Rotation2d DPAD_DOWN_ANGLE  =  Rotation2d.fromDegrees(180);//rotation setpoint for dpad down button
    public static final Rotation2d DPAD_LEFT_ANGLE  =  Rotation2d.fromDegrees(90);//rotation setpoint for dpad left button
    public static final Rotation2d DPAD_RIGHT_ANGLE =  Rotation2d.fromDegrees(270);//rotation setpoint for dpad right button

    //usb port of driver controller, remember to assign controller to port in driverstation
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 1;
  }
  public static class drive {
    public static final double MAX_SPEED = Units.feetToMeters(15); //max drive speed, m/s

    public static final boolean STARTING_FOD = true;
    public static final boolean STARTING_DIRECT_ANGLE = false;
  }

  public static final class AutoConstants
  {
    //PID constants for auto
    public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
    public static final PIDConstants ANGLE_PID       = new PIDConstants(0.4, 0, 0.01);
  }
}
