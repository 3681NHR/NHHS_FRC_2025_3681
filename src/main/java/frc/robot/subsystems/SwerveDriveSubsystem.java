package frc.robot.subsystems;

import frc.robot.Constants;

import java.io.File;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    drive.setCosineCompensator(!RobotBase.isSimulation());
    drive.setChassisDiscretization(true, 0.02);
    drive.setHeadingCorrection(false);
    drive.setMotorIdleMode(false);
    //drive.pushOffsetsToEncoders();

    //correct skew caused when rotating and translating at the same time
    //drive.setAngularVelocityCompensation(true, true, 0.1);

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
        drive.getMaximumVelocity())
      );
    } else {
      drive.setHeadingCorrection(false);// normaly false and needs to be false for simultaion
      
        // Make the robot move
        drive.drive(new Translation2d(
          translationX.getAsDouble() * drive.getMaximumVelocity(),
          translationY.getAsDouble() * drive.getMaximumVelocity()),
          rotateX.getAsDouble() * drive.getMaximumAngularVelocity(),
          fieldOriented.getAsBoolean(),
          false
        );
    }
  });
    
  }

  /**
   * Setup AutoBuilder for PathPlanner.
   */
  public void setupPathPlanner()
  {
    AutoBuilder.configureHolonomic(
        this::getPose,
        this::resetOdometry,
        this::getRobotVelocity,
        this::setChassisSpeeds,
        new HolonomicPathFollowerConfig(
                                         Constants.AutoConstants.TRANSLATION_PID,
                                         Constants.AutoConstants.ANGLE_PID,
                                         4.5,
                                         // Max module speed, in m/s
                                         drive.swerveDriveConfiguration.getDriveBaseRadiusMeters(),
                                         // Drive base radius in meters. Distance from robot center to furthest module.
                                         new ReplanningConfig()
        ),
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
          var alliance = DriverStation.getAlliance();
          return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
        },
        this // Reference to this subsystem to set requirements
                                  );
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
