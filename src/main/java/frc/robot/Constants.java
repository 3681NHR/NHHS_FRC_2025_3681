// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

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
  public static final double ENDGAME_TIME = 20;//time remaining in teleop when endgame starts

  
  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag

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

}
