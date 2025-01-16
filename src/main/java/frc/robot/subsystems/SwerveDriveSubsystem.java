package frc.robot.subsystems;

import frc.robot.Constants;

import java.io.File;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

/** swervedrive using YAGSL, this allows configuration using json files
 * YAGSL also handles kinematics and odometry for the drive
 */
public class SwerveDriveSubsystem extends SubsystemBase {
  
  private final SwerveDrive drive;

  private boolean visionOn = false;
  private Vision vision;

  public SwerveDriveSubsystem(File directory) {
    
    try
    {
      drive = new SwerveParser(directory).createSwerveDrive(Constants.drive.MAX_SPEED);
    } catch (Exception e)
    {
      throw new RuntimeException(e);
    }
    //cosine compensator is very helpfull, but works weird in simulation
    drive.setCosineCompensator(RobotBase.isReal());
    drive.setChassisDiscretization(true, 0.02);
    drive.setHeadingCorrection(false);
    drive.setMotorIdleMode(false);
    drive.setAngularVelocityCompensation(true, true, -0.1);

    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.MACHINE;

    setupPathPlanner();
    if (visionOn){
      drive.stopOdometryThread();
      setupPhotonVision();
    }
  }

  /**
   * Setup the photon vision class.
   */
  public void setupPhotonVision()
  {
    vision = new Vision();
  }

  @Override
  public void periodic() {
    if(visionOn){
      drive.updateOdometry();
      vision.updatePoseEstimation(drive);
    }
  }

  @Override
  public void simulationPeriodic() {
    drive.setCosineCompensator(false);
    drive.setHeadingCorrection(false);
  }

  /**
   * Command to drive the robot using translative and rotational values
   * <p> if angle is true, uses heading as setppoint for angle pid
   * <p> if angle is false, used headingX for anguler velocity
   *
   * @param translationX Translation in the X direction
   * @param translationY Translation in the Y direction
   * @param headingX     Heading X to calculate angle of the joystick. anguler velocity when angle is true
   * @param headingY     Heading Y to calculate angle of the joystick. unused when angle is true
   * @return Drive command.
   */
  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier rotateX, DoubleSupplier rotateY, BooleanSupplier directAngle, BooleanSupplier fieldOriented)
  {
    return run(() -> {

    if(directAngle.getAsBoolean()){
      drive.setHeadingCorrection(RobotBase.isReal()); // Normally you would want heading correction for this kind of control.
    
      drive.driveFieldOriented(drive.swerveController.getTargetSpeeds(
        translationX.getAsDouble(), 
        translationY.getAsDouble(),
        rotateX.getAsDouble(),
        rotateY.getAsDouble(),
        drive.getOdometryHeading().getRadians(),
        drive.getMaximumChassisVelocity())
      );
    } else {
      drive.setHeadingCorrection(false);// normaly false and needs to be false for simultaion
      
      ChassisSpeeds s = new ChassisSpeeds(
        translationY.getAsDouble() * drive.getMaximumChassisVelocity(),
        translationX.getAsDouble() * drive.getMaximumChassisVelocity(),
        rotateX.getAsDouble() * drive.getMaximumChassisAngularVelocity());
      
      if(fieldOriented.getAsBoolean()){
        s = ChassisSpeeds.fromFieldRelativeSpeeds(s, drive.getOdometryHeading());
      }
      drive.drive(s);
    }

    });
  }

  /**
   * Setup AutoBuilder for PathPlanner.
   */
  public void setupPathPlanner()
  {
    // Load the RobotConfig from the GUI settings. You should probably
    // store this in your Constants file
    RobotConfig config;
    try
    {
      config = RobotConfig.fromGUISettings();

      final boolean enableFeedforward = true;
      // Configure AutoBuilder last
      AutoBuilder.configure(
          this::getPose,
          // Robot pose supplier
          this::resetOdometry,
          // Method to reset odometry (will be called if your auto has a starting pose)
          this::getRobotVelocity,
          // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
          (speedsRobotRelative, moduleFeedForwards) -> {
            if (enableFeedforward)
            {
              drive.drive(
                  speedsRobotRelative,
                  drive.kinematics.toSwerveModuleStates(speedsRobotRelative),
                  moduleFeedForwards.linearForces()
                               );
            } else
            {
              drive.setChassisSpeeds(speedsRobotRelative);
            }
          },
          // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
          new PPHolonomicDriveController(
              // PPHolonomicController is the built in path following controller for holonomic drive trains
              new PIDConstants(1.0, 0.0, 0.0),
              // Translation PID constants
              new PIDConstants(0.4, 0.0, 0.01)
              // Rotation PID constants
          ),
          config,
          // The robot configuration
          () -> {
            // Boolean supplier that controls when the path will be mirrored for the red alliance
            // This will flip the path being followed to the red side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent())
            {
              return alliance.get() == DriverStation.Alliance.Red;
            }
            return false;
          },
          this
          // Reference to this subsystem to set requirements
          );

    } catch (Exception e)
    {
      // Handle exception as needed
      e.printStackTrace();
    }

    //Preload PathPlanner Path finding
    // IF USING CUSTOM PATHFINDER ADD BEFORE THIS LINE
    PathfindingCommand.warmupCommand().schedule();
  }
  
  public ChassisSpeeds getRobotVelocity(){return drive.getRobotVelocity();}
  
  public void setChassisSpeeds(ChassisSpeeds chassisSpeeds){drive.setChassisSpeeds(chassisSpeeds);}
  
  public Pose2d getPose(){return drive.getPose();}
  /**
   * reset odometry to given pose2d
   * @param initialHolonomicPose
   */
  public void resetOdometry(Pose2d initialHolonomicPose){
    drive.resetOdometry(initialHolonomicPose);
  }

  /** 
   * align wheels inward to make the robot very hard to move, effectivly locking it in place
   */
  public void lock(){
    drive.lockPose();
  }
  /**
   * Resets the gyro angle to zero and resets odometry to the same position, but facing toward 0.
   */
  public void zeroGyro()
  {
    drive.zeroGyro();
  }

}
