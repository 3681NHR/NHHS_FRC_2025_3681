package frc.robot.subsystems.vision;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.CameraIO.TargetObservation;
import static frc.robot.constants.VisionConstants.*;

import java.util.LinkedList;
import java.util.List;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
  private final CameraIO[] io;
  private final CameraIOInputsAutoLogged[] inputs;
  private final Alert[] disconnectedAlerts;

  private VisionEstimate[] latestEstimateRaw;
  private VisionEstimate[] latestEstimateFinal = latestEstimateRaw;

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
              "Camera: " + io[i].getName() == null ? Integer.toString(i) : io[i].getName() + " is disconnected.", AlertType.kError);
    }
  }

  @Override
  public void periodic() {
    for (int i = 0; i < io.length; i++) {
      io[i].updateInputs(inputs[i]);
      Logger.processInputs("Vision/" + CAMERA_NAMES[i], inputs[i]);
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
                || observation.pose().getX() < -1.0
                || observation.pose().getX() > APRILTAG_LAYOUT.getFieldLength()+1
                || observation.pose().getY() < -1.0
                || observation.pose().getY() > APRILTAG_LAYOUT.getFieldWidth()+1;

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
        Math.pow(observation.averageTagDistance(), 3.0) / observation.tagCount();
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
      double[][] stdDevs = new double[estimates.size()][3];
      for(int i=0; i<stdDevs.length; i++){
        stdDevs[i] = estimates.get(i).visionMeasurementStdDevs.getData();
      }
      Logger.recordOutput(
        "Vision/Camera: " + io[cameraIndex].getName() == null ? Integer.toString(cameraIndex) : io[cameraIndex].getName() + "/stdDevs",
        stdDevs);

      allTagPoses.addAll(tagPoses);
      allRobotPoses.addAll(robotPoses);
      allRobotPosesAccepted.addAll(robotPosesAccepted);
      allRobotPosesRejected.addAll(robotPosesRejected);
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
      "Vision/Summary/tags", allTagPoses.size());
    Logger.recordOutput(
        "Vision/Summary/RobotPoses", allRobotPoses.toArray(new Pose3d[allRobotPoses.size()]));
    Logger.recordOutput(
        "Vision/Summary/RobotPosesAccepted",
        allRobotPosesAccepted.toArray(new Pose3d[allRobotPosesAccepted.size()]));
    Logger.recordOutput(
        "Vision/Summary/RobotPosesRejected",
        allRobotPosesRejected.toArray(new Pose3d[allRobotPosesRejected.size()]));
    double[][] stdDevs = new double[allEstimates.size()][3];
    for(int i=0; i<stdDevs.length; i++){
      stdDevs[i] = allEstimates.get(i).visionMeasurementStdDevs.getData();
    }
    Logger.recordOutput("Vision/Summary/stdDevs", stdDevs);

    latestEstimateRaw = allEstimates.stream().toArray(VisionEstimate[]::new);
    latestEstimateFinal = latestEstimateRaw;

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

  public Optional<Double> getYaw(int tagID){
    Optional<Double> yaw = Optional.empty();
    if(tagID < 0){
      for(int i=0; i<inputs.length; i++){
        yaw = Optional.of(inputs[i].latestTargetObservation.tx().getRadians() + io[i].getRobotToCamera().getRotation().getZ());
      }
    } else {
      for(int i=0; i<inputs.length; i++){
        for(TargetObservation t : inputs[i].targets){//FIXME
          if(t != null){
            if(t.ID() == tagID){
              yaw = Optional.of(t.tx().getRadians() + io[i].getRobotToCamera().getRotation().getZ());
            }
          }
        }
      }
    }
    return yaw;
  }

}