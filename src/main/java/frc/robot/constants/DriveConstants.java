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

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import frc.utils.PIDGains; 

public class DriveConstants {
    public static final PIDGains.PID ANGLE_PID = new PIDGains.PID(0.5, 0.0, 0.025);
    public static final PIDGains.PID ANGLE_PID_SIM = new PIDGains.PID(0.5, 0.0, 0.025);
    public static final double ANGLE_MAX_VELOCITY = 11.2;
    public static boolean USE_VISION = true;

    public static final PIDGains.PID AUTO_ANGLE_PID = new PIDGains.PID(8, 0.0, 0.0);//TODO tune
    public static final PIDGains.PID AUTO_ANGLE_PID_SIM = new PIDGains.PID(5, 0.0, 0.0);
    
    public static final PIDGains.PID TRANS_PID = new PIDGains.PID(5, 0.0, 0.0);//TODO tune
    public static final PIDGains.PID TRANS_PID_SIM = new PIDGains.PID(6, 0.0, 0.0);
    public static final double MAX_SPEED = 4.7;//TODO tune

    public static final double ODOMETRY_FREQ = 100.0; // Hz
    public static final double WIDTH = Units.inchesToMeters(22);
    public static final double LENGTH = Units.inchesToMeters(25);
    public static final double RADIUS = Math.hypot(WIDTH / 2.0, LENGTH / 2.0);
  public static final Translation2d[] MODULE_POSITIONS =
      new Translation2d[] {
        new Translation2d(WIDTH / 2.0, LENGTH / 2.0),
        new Translation2d(WIDTH / 2.0, -LENGTH / 2.0),
        new Translation2d(-WIDTH / 2.0, LENGTH / 2.0),
        new Translation2d(-WIDTH / 2.0, -LENGTH / 2.0)
      };

  // Zeroed rotation values for each module, see setup instructions
  public static final Rotation2d FL_ZERO = new Rotation2d(0.0);
  public static final Rotation2d FR_ZERO = new Rotation2d(0.0);
  public static final Rotation2d BL_ZERO = new Rotation2d(0.0);
  public static final Rotation2d BR_ZERO = new Rotation2d(0.0);

  // Device CAN IDs
  public static final int GYRO_ID = 30;

  public static final int FL_DRIVE_ID = 11;
  public static final int FR_DRIVE_ID = 12;
  public static final int BL_DRIVE_ID = 13;
  public static final int BR_DRIVE_ID = 14;

  public static final int FL_TURN_ID = 21;
  public static final int FR_TURN_ID = 22;
  public static final int BL_TURN_ID = 23;
  public static final int BR_TURN_ID = 24;

  // Drive motor configuration
  public static final boolean DRIVE_INVERT = true;
  public static final int DRIVE_MAX_CURRENT = 30;
  public static final double WHEEL_RAD = Units.inchesToMeters(2);
  public static final double DRIVE_REDUCTION = 6.75;
  public static final DCMotor DRIVE_GEARBOX = DCMotor.getNEO(1);

  // Drive encoder configuration
  public static final double DRIVE_ENCODER_POS_FACTOR =
      2 * Math.PI / DRIVE_REDUCTION; // Rotor Rotations -> Wheel Radians
  public static final double DRIVE_ENCODER_VEL_FACTOR =
      (2 * Math.PI) / 60.0 / DRIVE_REDUCTION; // Rotor RPM -> Wheel Rad/Sec

  // Drive PID configuration
  public static final PIDGains.PID DRIVE_PID = new PIDGains.PID(0.01, 0.0, 0.0);//TODO should tune
  public static final PIDGains.SimpleFF DRIVE_FF = new PIDGains.SimpleFF(0.11, 0.13, 0.1);//TODO needs sysid
  
  public static final PIDGains.PID DRIVE_PID_SIM = new PIDGains.PID(0.01, 0.0, 0.0);
  public static final PIDGains.SimpleFF DRIVE_FF_SIM = new PIDGains.SimpleFF(0.11, 0.13, 0.1);

  // Turn motor configuration
  public static final boolean TURN_INVERT = true;
  public static final int TURN_CURRENT_LIM = 10;
  public static final double TURN_REDUCTION = 21.428;
  public static final DCMotor TURN_GEARBOX = DCMotor.getNEO(1);

  // Turn encoder configuration
  public static final boolean TURN_ENCODER_INVERT = true;
  public static final double TURN_ENCODER_POS_FACTOR = 2 * Math.PI; // Rotations -> Radians
  public static final double TURN_ENCODER_VEL_FACTOR = (2 * Math.PI) / 60.0; // RPM -> Rad/Sec

  // Turn PID configuration
  public static final PIDGains.ProfiledPID TURN_PID = new PIDGains.ProfiledPID(7.5, 0.0, 0.0, Math.PI*8, Math.PI*80);//TODO needs tuning
  public static final PIDGains.SimpleFF TURN_FF = new PIDGains.SimpleFF(0.125, 0.0, 0.0);//TODO needs tuning and sysid
  
  public static final PIDGains.ProfiledPID TURN_PID_SIM = new PIDGains.ProfiledPID(12.5, 0.0, 0.5, Math.PI*8, Math.PI*80);
  public static final PIDGains.SimpleFF TURN_FF_SIM = new PIDGains.SimpleFF(0.015, 0.0, 0.0);
  public static final double TURN_MIN_POS = 0; // Radians
  public static final double TURN_MAX_POS = 2 * Math.PI; // Radians

  // PathPlanner configuration
  public static final double MASS = 60;
  public static final double MOI = 6;
  public static final double COF = 2.31421199;
  public static final RobotConfig PP_CONFIG =
      new RobotConfig(
          MASS,
          MOI,
          new ModuleConfig(
              WHEEL_RAD,
              MAX_SPEED,
              COF,
              DRIVE_GEARBOX.withReduction(DRIVE_REDUCTION),
              DRIVE_MAX_CURRENT,
              1),
          MODULE_POSITIONS);

    //sysid
    public static final Voltage DRIVE_SYSID_VSTEP = Volts.of(4);
    public static final Velocity<VoltageUnit> DRIVE_SYSID_VRAMP = Volts.of(.5).per(Second);
    public static final Time DRIVE_SYSID_TIMEOUT = Seconds.of(10);

    
    public static final Voltage TURN_SYSID_VSTEP = Volts.of(4);
    public static final Velocity<VoltageUnit> TURN_SYSID_VRAMP = Volts.of(.5).per(Second);
    public static final Time TURN_SYSID_TIMEOUT = Seconds.of(10);

    public static final double ANGULAR_VELOCITY_COEFFICIENT = 0.02;

    public static final double TIP_RECOVERY_THRESHOLD = Units.degreesToRadians(10);

    public static class presets{
        public static final Rotation2d CLIMB = Rotation2d.fromDegrees(180);
        public static final Pose2d WEST_STATION = new Pose2d(new Translation2d(), Rotation2d.fromDegrees(30));
        public static final Pose2d EAST_STATION = new Pose2d(new Translation2d(), Rotation2d.fromDegrees(142));
        public static final Pose2d PROSSESOR = new Pose2d(new Translation2d(), Rotation2d.fromDegrees(270));
    }
}