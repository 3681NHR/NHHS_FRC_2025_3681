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

  
  public static class robotDims{
    public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
    public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
    
    public static final Translation3d ROBOT_SIZE = new Translation3d(
                                                                    Units.inchesToMeters(26),
                                                                    Units.inchesToMeters(30),
                                                                    Units.feetToMeters(6)
                                                                    );
  }
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms spark max velocity lag

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

    public static final boolean STARTING_FOD = true;//is FOD enabled on start
    public static final boolean STARTING_DIRECT_ANGLE = false;// is direct angle enabled on start
  }

  public static class arm {
    public class gains{
      //pid gains
      public static final double EXTENTION_KP = 0;
      public static final double EXTENTION_KI = 0;
      public static final double EXTENTION_KD = 0;
      //ff gains
      public static final double EXTENTION_KS = 0;
      public static final double EXTENTION_KG = 0;
      public static final double EXTENTION_KV = 0;

      //pid gains
      public static final double ANGLE_KP = 0;
      public static final double ANGLE_KI = 0;
      public static final double ANGLE_KD = 0;
      //ff gains
      public static final double ANGLE_KS = 0;
      public static final double ANGLE_KG = 0;
      public static final double ANGLE_KV = 0;

      public static final double ANGLE_FACTOR = 360;
      public static final double EXTENTION_FACTOR = 1;

      public static final boolean ANGLE_ENCODER_INVERTED = false;
      public static final boolean EXTENTION_ENCODER_INVERTED = false;
    }
    public static class ids {
      public static final int EXTENTION_1_ID = 0;
      public static final int EXTENTION_2_ID = 0;

      public static final int ANGLE_1_ID = 0;
      public static final int ANGLE_2_ID = 0;

      public static final int ANGLE_ENCODER = 0;
      public static final int EXTENTION_ENCODER = 0;
    }
    public static class limits {
      public static final double MAX_EXTENTION = Units.inchesToMeters(70.2);//meters
      public static final double MIN_EXTENTION = Units.inchesToMeters(26.8);//meters
      
      public static final double MAX_ANGLE = Integer.MAX_VALUE;//degrees
      public static final double MIN_ANGLE = Integer.MIN_VALUE;//degrees

      public static final double EXTENTION_LIMIT = Integer.MAX_VALUE;//max distance the arm is allowed to extend past the perimiter, meters
    }
    //motor controller settings are set in frc/robot/subsystems/ArmSubsystem.java
    public static final double ANGLE_OFFSET = 0; //added to encoder value
    public static final double EXTENTION_OFFSET = 0; //added to encoder value

    public static final Translation3d OFFSET = new Translation3d( Units.inchesToMeters(0),
                                                                  Units.inchesToMeters(-12),
                                                                  Units.inchesToMeters(12)
    );
  }

}
