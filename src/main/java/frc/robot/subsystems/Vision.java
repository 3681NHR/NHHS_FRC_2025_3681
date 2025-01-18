package frc.robot.subsystems;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import swervelib.SwerveDrive;

public class Vision
{
  private AprilTagFieldLayout layout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
  
  private final double maxError = 5;
  private final double maxRotError = 90;

  private Camera[] cameras = {
    new Camera.builder()
              .withPosition(new Translation3d(Units.inchesToMeters(14), 0, Units.inchesToMeters(6)))
              .withAngle(new Rotation3d())
              .withCamera(new PhotonCamera("front"))
              .withField(layout)
              .build(),
    
  };

  public Vision(){
    
  }
  public EstimatedRobotPose[] getEstimatedRobotPoses(Pose3d robotPose){
    EstimatedRobotPose[] poses = new EstimatedRobotPose[cameras.length];
    for(int i=0; i<poses.length; i++){
      Camera c = cameras[i];
      if(RobotBase.isReal()){
        poses[i] = (c.update());
      } else {
        poses[i] = c.updateSim(robotPose);
      }
    }
    return poses;
  }
  public void updatePoseEstimation(SwerveDrive drive){
    for(EstimatedRobotPose e : getEstimatedRobotPoses(new Pose3d(drive.getMapleSimDrive().get().getSimulatedDriveTrainPose()))){
      if(e != null){
        if(checkPoseError(e.estimatedPose.toPose2d(), drive.getPose())){
          System.err.println("pose error over "+maxError+" meters or over "+maxRotError+" degrees. WARNING: mesurement is still being added to odometry");
        }
        drive.addVisionMeasurement(e.estimatedPose.toPose2d(), e.timestampSeconds);
      }
    }
  }
  private boolean checkPoseError(Pose2d a, Pose2d b) {
    return a.getTranslation().getDistance(b.getTranslation()) >= maxError || a.getRotation().minus(b.getRotation()).getDegrees() > maxRotError;
  }
    
  
}