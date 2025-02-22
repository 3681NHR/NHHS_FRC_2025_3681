package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.*;
import static frc.robot.constants.DriveConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.PathPlannerLogging;

import choreo.trajectory.SwerveSample;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.RobotMode;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionEstimate;
import frc.utils.LoggedField2d;
import frc.utils.SparkOdometryThread;

import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Drive extends SubsystemBase {
  public static final Lock odometryLock = new ReentrantLock();
  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final Module[] modules = new Module[4]; // FL, FR, BL, BR
  private final SysIdRoutine driveSysId;
  private final SysIdRoutine steerSysId;
  private final SysIdRoutine angleSysId;
  private final Vision vision;
  private LoggedField2d field = new LoggedField2d();
  private final Alert gyroDisconnectedAlert =
      new Alert("Disconnected gyro, using kinematics as fallback.", AlertType.kError);

  private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(MODULE_POSITIONS);
  private Rotation2d rawGyroRotation = new Rotation2d();
  private SwerveModulePosition[] lastModulePositions = // For delta tracking
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };
  private SwerveDrivePoseEstimator poseEstimator =
      new SwerveDrivePoseEstimator(kinematics, rawGyroRotation, lastModulePositions, Constants.STARTING_POSE);

  public Drive(
      GyroIO gyroIO,
      ModuleIO flModuleIO,
      ModuleIO frModuleIO,
      ModuleIO blModuleIO,
      ModuleIO brModuleIO,
      Vision vision) {
    this.vision = vision;
    this.gyroIO = gyroIO;
    modules[0] = new Module(flModuleIO, 0);
    modules[1] = new Module(frModuleIO, 1);
    modules[2] = new Module(blModuleIO, 2);
    modules[3] = new Module(brModuleIO, 3);

    // Usage reporting for swerve template
    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_AdvantageKit);

    // Start odometry thread
    SparkOdometryThread.getInstance().start();

    // Configure AutoBuilder for PathPlanner
    AutoBuilder.configure(
        this::getPose,
        this::setPose,
        this::getChassisSpeeds,
        this::runVelocity,
        new PPHolonomicDriveController(
            new PIDConstants(6.0, 0.0, 0.0), new PIDConstants(6, 0.0, 0.0)),
        PP_CONFIG,
        () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
        this);
    PathPlannerLogging.setLogActivePathCallback(
        (activePath) -> {
          Logger.recordOutput(
              "Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));  
          field.getObject("PP/activePath").setPoses(activePath);
        });
    PathPlannerLogging.setLogTargetPoseCallback(
        (targetPose) -> {
          Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
          field.getObject("PP/targetpose").setPoses(targetPose);
        });

    // Configure SysId
    driveSysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
              DRIVE_SYSID_VRAMP,
              DRIVE_SYSID_VSTEP,
              DRIVE_SYSID_TIMEOUT,
                (state) -> Logger.recordOutput("Drive/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> runCharacterization(voltage.in(Volts)), null, this));
    steerSysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                TURN_SYSID_VRAMP,
                TURN_SYSID_VSTEP,
                TURN_SYSID_TIMEOUT,
                (state) -> Logger.recordOutput("Drive/SteerSysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> runSteerCharacterization(voltage.in(Volts)), null, this));
    angleSysId =
    new SysIdRoutine(
        new SysIdRoutine.Config(
            null,
            null,
            Seconds.of(10),
            (state) -> Logger.recordOutput("Drive/AngleSysIdState", state.toString())),
        new SysIdRoutine.Mechanism(
            (voltage) -> runAngleCharacterization(voltage.in(Volts)), null, this));

    YAGSLWidget.maxAngularVelocity = getMaxAngularSpeedRadPerSec();
    YAGSLWidget.maxSpeed = getMaxLinearSpeedMetersPerSec();
    YAGSLWidget.moduleCount = 4;
    YAGSLWidget.sizeFrontBack = LENGTH;
    YAGSLWidget.sizeLeftRight = WIDTH;
    YAGSLWidget.wheelLocations = new double[8];

    for (int i=0; i>MODULE_POSITIONS.length; i += 2)
      {
        Translation2d t = MODULE_POSITIONS[i];
        YAGSLWidget.wheelLocations[i * 2] = t.getX();
        YAGSLWidget.wheelLocations[(i * 2) + 1] = t.getY();
      }
  }

  @Override
  public void periodic() {
    odometryLock.lock(); // Prevents odometry updates while reading data
    gyroIO.updateInputs(gyroInputs);
    Logger.processInputs("Drive/Gyro", gyroInputs);
    for (var module : modules) {
      module.periodic();
    }
    if(USE_VISION){
      for(VisionEstimate e : vision.getPose()){
        poseEstimator.addVisionMeasurement(
            e.pose, e.timestampSeconds, e.visionMeasurementStdDevs);
      }
    }
    odometryLock.unlock();

    // Stop moving when disabled
    if (DriverStation.isDisabled()) {
      for (var module : modules) {
        module.stop();
      }
    }

    // Log empty setpoint states when disabled
    if (DriverStation.isDisabled()) {
      Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
      Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
    }

    // Update odometry
    double[] sampleTimestamps =
        modules[0].getOdometryTimestamps(); // All signals are sampled together
    int sampleCount = sampleTimestamps.length;
    for (int i = 0; i < sampleCount; i++) {
      // Read wheel positions and deltas from each module
      SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
      SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
      for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
        modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
        moduleDeltas[moduleIndex] =
            new SwerveModulePosition(
                modulePositions[moduleIndex].distanceMeters
                    - lastModulePositions[moduleIndex].distanceMeters,
                modulePositions[moduleIndex].angle);
        lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
      }

      // Update gyro angle
      if (gyroInputs.connected) {
        // Use the real gyro angle
        rawGyroRotation = gyroInputs.odometryYawPositions[i];
      } else {
        // Use the angle delta from the kinematics and module deltas
        Twist2d twist = kinematics.toTwist2d(moduleDeltas);
        rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
      }

      // Apply update
      poseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, modulePositions);
    }

    // Update gyro alert
    gyroDisconnectedAlert.set(!gyroInputs.connected && Constants.MODE != RobotMode.SIM);

    YAGSLWidget.measuredStatesObj = getModuleStates();
    YAGSLWidget.measuredChassisSpeedsObj = getChassisSpeeds();
    YAGSLWidget.robotRotationObj = getRotation();

    YAGSLWidget.updateData();

    field.setRobotPose(getPose());

    SmartDashboard.putData("field", field);
  }

  /**
   * Runs the drive at the desired velocity.
   *
   * @param speeds Speeds in meters/sec
   */
  public void runVelocity(ChassisSpeeds speeds) {
    // Calculate module setpoints
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
    SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, MAX_SPEED);

    YAGSLWidget.desiredChassisSpeedsObj = discreteSpeeds;
    // Log unoptimized setpoints
    Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
    Logger.recordOutput("SwerveChassisSpeeds/Setpoints", discreteSpeeds);
    Logger.recordOutput("SwerveChassisSpeeds/SetpointAngularVel", discreteSpeeds.omegaRadiansPerSecond);
    
    // Send setpoints to modules
    for (int i = 0; i < 4; i++) {
      modules[i].runSetpoint(setpointStates[i]);
    }
    
    YAGSLWidget.desiredStatesObj = setpointStates;
    // Log optimized setpoints (runSetpoint mutates each state)
    Logger.recordOutput("SwerveStates/SetpointsOptimized", setpointStates);
  }

  /** Runs the drive in a straight line with the specified drive output. */
  public void runCharacterization(double output) {
    for (int i = 0; i < 4; i++) {
      modules[i].runCharacterization(output);
    }
  }
  /** spins modules*/
  public void runSteerCharacterization(double output) {
    for (int i = 0; i < 4; i++) {
      modules[i].runSteerCharacterization(output);
    }
  }
  /** spins robot*/
  public void runAngleCharacterization(double output) {
    runVelocity(new ChassisSpeeds(0, 0, output));
  }

  /** Stops the drive. */
  public void stop() {
    runVelocity(new ChassisSpeeds());
  }

  /**
   * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
   * return to their normal orientations the next time a nonzero velocity is requested.
   */
  public void stopWithX() {
    Rotation2d[] headings = new Rotation2d[4];
    for (int i = 0; i < 4; i++) {
      headings[i] = MODULE_POSITIONS[i].getAngle();
    }
    kinematics.resetHeadings(headings);
    stop();
  }

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0))
        .withTimeout(1.0)
        .andThen(driveSysId.quasistatic(direction));
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(driveSysId.dynamic(direction));
  }

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command steerSysIdQuasistatic(SysIdRoutine.Direction direction) {
    return run(() -> runSteerCharacterization(0.0))
        .withTimeout(1.0)
        .andThen(steerSysId.quasistatic(direction));
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command steerSysIdDynamic(SysIdRoutine.Direction direction) {
    return run(() -> runSteerCharacterization(0.0)).withTimeout(1.0).andThen(steerSysId.dynamic(direction));
  }
  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command angleSysIdQuasistatic(SysIdRoutine.Direction direction) {
    return run(() -> runAngleCharacterization(0.0))
        .withTimeout(1.0)
        .andThen(angleSysId.quasistatic(direction));
  }
  /** Returns a command to run a dynamic test in the specified direction. */
  public Command angleSysIdDynamic(SysIdRoutine.Direction direction) {
    return run(() -> runAngleCharacterization(0.0)).withTimeout(1.0).andThen(angleSysId.dynamic(direction));
  }

  /** Returns the module states (turn angles and drive velocities) for all of the modules. */
  @AutoLogOutput(key = "SwerveStates/Measured")
  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    
    return states;
  }

  /** Returns the module positions (turn angles and drive positions) for all of the modules. */
  private SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] states = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getPosition();
    }
    return states;
  }

  /** Returns the measured chassis speeds of the robot. */
  @AutoLogOutput(key = "SwerveChassisSpeeds/Measured")
  private ChassisSpeeds getChassisSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }


  /** Returns the current odometry pose. */
  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /** Returns the current odometry rotation. */
  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  /** Resets the current odometry pose. */
  public void setPose(Pose2d pose) {
    poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
  }

  /** Returns the maximum linear speed in meters per sec. */
  public double getMaxLinearSpeedMetersPerSec() {
    return MAX_SPEED;
  }

  /** Returns the maximum angular speed in radians per sec. */
  public double getMaxAngularSpeedRadPerSec() {
    return MAX_SPEED / RADIUS;
  }

  public void resetGyro(double headingRad){
    poseEstimator.resetPose(new Pose2d(getPose().getX(), getPose().getY(), new Rotation2d(headingRad)));
  }

  public double getAngulerVelocity(){
    return gyroInputs.yawVelocityRadPerSec;
  }

}