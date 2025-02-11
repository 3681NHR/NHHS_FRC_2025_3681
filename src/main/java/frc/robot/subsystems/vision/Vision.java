package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.VisionConstants;
import static frc.robot.constants.VisionConstants.*;

import java.util.LinkedList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
  private final CameraIO[] io;
  private final CameraIOInputsAutoLogged[] inputs;
  private final Alert[] disconnectedAlerts;

  private VisionEstimate[] latestEstimateRaw;
  private VisionEstimate[] latestEstimateFinal = latestEstimateRaw;

  private LinearFilter xFilterSP = LinearFilter.singlePoleIIR(0.1, 0.2);
  private LinearFilter yFilterSP = LinearFilter.singlePoleIIR(0.1, 0.2);
  private LinearFilter tFilterSP = LinearFilter.singlePoleIIR(0.1, 0.2);

  private LinearFilter xFilterMean = LinearFilter.movingAverage(5);
  private LinearFilter yFilterMean = LinearFilter.movingAverage(5);
  private LinearFilter tFilterMean = LinearFilter.movingAverage(5);

  private MedianFilter xFilterMedian = new MedianFilter(5);
  private MedianFilter yFilterMedian = new MedianFilter(5);
  private MedianFilter tFilterMedian = new MedianFilter(5);

  private SlewRateLimiter xFilterRate = new SlewRateLimiter(10);
  private SlewRateLimiter yFilterRate = new SlewRateLimiter(10);
  private SlewRateLimiter tFilterRate = new SlewRateLimiter(10);

  public Vision(CameraIO... io) {
    this.io = io;

    // Initialize inputs
    this.inputs = new CameraIOInputsAutoLogged[io.length];
    for (int i = 0; i < inputs.length; i++) {
      inputs[i] = new CameraIOInputsAutoLogged();
    }

    // Initialize disconnected alerts
    this.disconnectedAlerts = new Alert[io.length];
    for (int i = 0; i < inputs.length; i++) {
      disconnectedAlerts[i] =
          new Alert(
              "Camera: " + io[i].getName() == null ? Integer.toString(i) : io[i].getName() + " is disconnected.", AlertType.kWarning);
    }
  }

  @Override
  public void periodic() {
    for (int i = 0; i < io.length; i++) {
      io[i].updateInputs(inputs[i]);
      Logger.processInputs("Vision/Camera: " + io[i].getName() == null ? Integer.toString(i) : io[i].getName(), inputs[i]);
    }

    // Initialize logging values
    List<Pose3d> allTagPoses = new LinkedList<>();
    List<Pose3d> allRobotPoses = new LinkedList<>();
    List<Pose3d> allRobotPosesAccepted = new LinkedList<>();
    List<Pose3d> allRobotPosesRejected = new LinkedList<>();
    List<VisionEstimate> allEstimates = new LinkedList<>();

    
    // Loop over cameras
    for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
      
      // Initialize logging values
      List<VisionEstimate> estimates = new LinkedList<>();
      List<Pose3d> tagPoses = new LinkedList<>();
      List<Pose3d> robotPoses = new LinkedList<>();
      List<Pose3d> robotPosesAccepted = new LinkedList<>();
      List<Pose3d> robotPosesRejected = new LinkedList<>();
      
      // Update disconnected alert
      disconnectedAlerts[cameraIndex].set(!inputs[cameraIndex].connected);

      // Add tag poses
      for (int tagId : inputs[cameraIndex].tagIds) {
        var tagPose = APRILTAG_LAYOUT.getTagPose(tagId);
        if (tagPose.isPresent()) {
          tagPoses.add(tagPose.get());
        }
      }

      // Loop over pose observations
      for (var observation : inputs[cameraIndex].poseObservations) {
        // Check whether to reject pose
        boolean rejectPose =
            observation.tagCount() == 0 // Must have at least one tag
                || observation.ambiguity() > MAX_AMBIGUITY // Cannot be too high ambiguity
                || Math.abs(observation.pose().getZ())
                    > MAX_Z_ERROR // Must have realistic Z coordinate

                // Must be within the field boundaries
                || observation.pose().getX() < 0.0
                || observation.pose().getX() > APRILTAG_LAYOUT.getFieldLength()
                || observation.pose().getY() < 0.0
                || observation.pose().getY() > APRILTAG_LAYOUT.getFieldWidth();

        // Add pose to log
        robotPoses.add(observation.pose());
        if (rejectPose) {
          robotPosesRejected.add(observation.pose());
        } else {
          robotPosesAccepted.add(observation.pose());
        }

        // Skip if rejected
        if (rejectPose) {
          continue;
        }

        double stdDevFactor =
        Math.pow(observation.averageTagDistance(), 2.0) / observation.tagCount();
        double linearStdDev = LIN_STD_DEV_BASELINE * stdDevFactor;
        double angularStdDev = ANG_STD_DEV_BASELINE * stdDevFactor;
        if (cameraIndex < CAM_STD_DEV_FACTORS.length) {
          linearStdDev *= CAM_STD_DEV_FACTORS[cameraIndex];
          angularStdDev *= CAM_STD_DEV_FACTORS[cameraIndex];
        } else {
          throw new RuntimeException("could not find std dev factors for camera index: " + cameraIndex);
        }

        estimates.add(new VisionEstimate(
          observation.pose().toPose2d(),
          observation.timestamp(),
          VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev)));

      }

      // Log camera data
      if (tagPoses.size() > 0) {
        Logger.recordOutput(
            "Vision/Camera: " + io[cameraIndex].getName() == null ? Integer.toString(cameraIndex) : io[cameraIndex].getName() + "/TagPoses",
            tagPoses.toArray(new Pose3d[0]));
      }
      if (robotPoses.size() > 0) {
        Logger.recordOutput(
            "Vision/Camera: " + io[cameraIndex].getName() == null ? Integer.toString(cameraIndex) : io[cameraIndex].getName() + "/AllRobotPoses",
            robotPoses.toArray(new Pose3d[0]));
      }
      if (robotPosesAccepted.size() > 0) {
        Logger.recordOutput(
            "Vision/Camera: " + io[cameraIndex].getName() == null ? Integer.toString(cameraIndex) : io[cameraIndex].getName() + "/RobotPosesAccepted",
            robotPosesAccepted.toArray(new Pose3d[0]));
      }
      if (robotPosesRejected.size() > 0) {
        Logger.recordOutput(
            "Vision/Camera: " + io[cameraIndex].getName() == null ? Integer.toString(cameraIndex) : io[cameraIndex].getName() + "/RobotPosesRejected",
            robotPosesRejected.toArray(new Pose3d[0]));
      }
      Logger.recordOutput(
        "Vision/Camera: " + io[cameraIndex].getName() == null ? Integer.toString(cameraIndex) : io[cameraIndex].getName() + "/stdDevs",
        estimates.stream().map((t) -> t.visionMeasurementStdDevs).toArray(Matrix[]::new));

      allTagPoses.addAll(tagPoses);
      allRobotPoses.addAll(robotPoses);
      allRobotPosesAccepted.addAll(robotPosesAccepted);
      allEstimates.addAll(estimates);

      tagPoses.clear();
      robotPoses.clear();
      robotPosesAccepted.clear();
      robotPosesRejected.clear();
      estimates.clear();
    }

    // Log summary data
    Logger.recordOutput(
        "Vision/Summary/TagPoses", allTagPoses.toArray(new Pose3d[allTagPoses.size()]));
    Logger.recordOutput(
        "Vision/Summary/RobotPoses", allRobotPoses.toArray(new Pose3d[allRobotPoses.size()]));
    Logger.recordOutput(
        "Vision/Summary/RobotPosesAccepted",
        allRobotPosesAccepted.toArray(new Pose3d[allRobotPosesAccepted.size()]));
    Logger.recordOutput(
        "Vision/Summary/RobotPosesRejected",
        allRobotPosesRejected.toArray(new Pose3d[allRobotPosesRejected.size()]));
    Logger.recordOutput("Vision/Summary/stdDevs", allEstimates.toArray(new Matrix[0]));

    latestEstimateRaw = allEstimates.stream().toArray(VisionEstimate[]::new);

    latestEstimateFinal = new VisionEstimate[latestEstimateRaw.length];

    for (int i=0; i<latestEstimateRaw.length; i++) {
      latestEstimateFinal[i] = new VisionEstimate(
        FilterPose(latestEstimateRaw[i].pose, VisionConstants.POSE_FILTER),
        latestEstimateRaw[i].timestampSeconds,
        latestEstimateRaw[i].visionMeasurementStdDevs
      );
    }

    allTagPoses.clear();
    allRobotPoses.clear();
    allRobotPosesAccepted.clear();
    allRobotPosesRejected.clear();
    allEstimates.clear();
  }

  public VisionEstimate[] getPose() {
    // Send vision observation
    return latestEstimateFinal;
  }

  private Pose2d FilterPose(Pose2d p, FilterStrategy strat){
    switch (strat) {      
      case RATE_LIM:
      return new Pose2d(
        new Translation2d(
          xFilterRate.calculate(p.getTranslation().getX()),
          yFilterRate.calculate(p.getTranslation().getY())
        ),
        new Rotation2d(tFilterRate.calculate(p.getRotation().getRadians()))
      );

      case SINGLE_POLE_IIR:
        return new Pose2d(
        new Translation2d(
          xFilterSP.calculate(p.getTranslation().getX()),
          yFilterSP.calculate(p.getTranslation().getY())
        ),
        new Rotation2d(tFilterSP.calculate(p.getRotation().getRadians()))
      );

      case MEAN:
      return new Pose2d(
        new Translation2d(
          xFilterMean.calculate(p.getTranslation().getX()),
          yFilterMean.calculate(p.getTranslation().getY())
        ),
        new Rotation2d(tFilterMean.calculate(p.getRotation().getRadians()))
      );

      case MEDIAN:
      return new Pose2d(
        new Translation2d(
          xFilterMedian.calculate(p.getTranslation().getX()),
          yFilterMedian.calculate(p.getTranslation().getY())
        ),
        new Rotation2d(tFilterMedian.calculate(p.getRotation().getRadians()))
      );
    
      default:
        return p;
    }
    
  }

 
}