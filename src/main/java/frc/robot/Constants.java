// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.subsystems.arm.ArmPosition;
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

  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }
  
  public static class robotDims{
    public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
    public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
    
    public static final Translation3d ROBOT_SIZE = new Translation3d(
                                                                    Units.inchesToMeters(26),
                                                                    Units.inchesToMeters(30),
                                                                    Units.feetToMeters(6)
                                                                    );
    public static final double ROBOT_HEIGHT_OFF_GROUND = 0.09;
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

    public static final double ARM_ANGLE_SENSITIVITY = 90*0.02;//deg/0.02sec
    public static final double ARM_EXTENTION_SENSITIVITY = 0.5*0.02;//m/0.02sec
    public static final double WRIST_ANGLE_SENSITIVITY = 90*0.02;//deg/0.02sec

    public static final double OPERATOR_TRIGGER_DEADBAND = 0.01;
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

      //pid gains
      public static final double WRIST_KP = 0;
      public static final double WRIST_KI = 0;
      public static final double WRIST_KD = 0;
      //ff gains
      public static final double WRIST_KS = 0;
      public static final double WRIST_KG = 0;
      public static final double WRIST_KV = 0;

      public static final double ANGLE_FACTOR = 360;
      public static final double EXTENTION_FACTOR = 1;
      public static final double WRIST_FACTOR = 360;

      public static final boolean ANGLE_ENCODER_INVERTED = false;
      public static final boolean EXTENTION_ENCODER_INVERTED = false;
      public static final boolean WRIST_ENCODER_INVERTED = false;
    }
    public static class ids {
      public static final int EXTENTION_1_ID = 41;
      public static final int EXTENTION_2_ID = 42;

      public static final int ANGLE_1_ID = 51;
      public static final int ANGLE_2_ID = 52;

      public static final int WRIST_ID = 61;

      public static final int WRIST_ENCODER = 1;
      public static final int ANGLE_ENCODER = 2;
      public static final int EXTENTION_ENCODER = 3;
    }
    public static class limits {
      public static final double MAX_EXTENTION = Units.inchesToMeters(70.2);//meters
      public static final double MIN_EXTENTION = Units.inchesToMeters(26.8);//meters
      
      public static final double MAX_ANGLE = Integer.MAX_VALUE;//degrees
      public static final double MIN_ANGLE = Integer.MIN_VALUE;//degrees

      public static final double MAX_WRIST = Integer.MAX_VALUE;//degrees
      public static final double MIN_WRIST = Integer.MIN_VALUE;//degrees

      public static final double EXTENTION_LIMIT = Integer.MAX_VALUE;//max distance the arm is allowed to extend past the perimiter, meters

      public static final double EXTENTION_DEADBAND = .05;//when current reading is within this much of the target, it is consitered "at setpoint", note the pid and ff will not stop within this range
      public static final double ANGLE_DEADBAND = 5;//when current reading is within this much of the target, it is consitered "at setpoint", note the pid and ff will not stop within this range
      public static final double WRIST_DEADBAND = 2.5;//when current reading is within this much of the target, it is consitered "at setpoint", note the pid and ff will not stop within this range
    
      public static final double EXTENTION_NEAR_RANGE = 0.25;//when current reading is within this much of the target, it is consitered "near setpoint"
      public static final double ANGLE_NEAR_RANGE = 10;//when current reading is within this much of the target, it is consitered "near setpoint"
      public static final double WRIST_NEAR_RANGE = 5;//when current reading is within this much of the target, it is consitered "near setpoint"
    }
    public static class positions{
      public static final ArmPosition CORAL_GROUND  = new ArmPosition(-4.3, 0, -63.3);
      public static final ArmPosition CORAL_STATION = new ArmPosition(44.3, 0, 0);
      public static final ArmPosition CORAL_L1      = new ArmPosition(24, 0, -53);
      public static final ArmPosition CORAL_L2      = new ArmPosition(103.3, 0, 122);
      public static final ArmPosition CORAL_L3      = new ArmPosition(95, 0, 115);
      public static final ArmPosition CORAL_L4      = new ArmPosition(97, 0, 138);
      public static final ArmPosition ALGE_L2       = new ArmPosition(0, 0, 0);
      public static final ArmPosition ALGE_L3       = new ArmPosition(0, 0, 0);
      public static final ArmPosition PROSESSOR     = new ArmPosition(0, 0, 0);

      public static final double ARM_1_LENGTH = Units.inchesToMeters(26.8);
      public static final double ARM_2_LENGTH = Units.inchesToMeters(26.8);
      public static final double ARM_3_LENGTH = Units.inchesToMeters(26.8);

      public static final double ARM_3_OFFSET = Units.inchesToMeters(26.8);
    }
    //motor controller settings are set in frc/robot/subsystems/ArmSubsystem.java
    public static final double ANGLE_OFFSET = 0; //added to encoder value
    public static final double WRIST_OFFSET = 0; //added to encoder value
    public static final double EXTENTION_OFFSET = 0; //added to encoder value

    public static final Translation3d OFFSET = new Translation3d( Units.inchesToMeters(-12),
                                                                  Units.inchesToMeters(0),
                                                                  Units.inchesToMeters(12)
    );
  }
}
