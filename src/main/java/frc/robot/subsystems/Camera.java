package frc.robot.subsystems;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Camera {
    private Rotation3d angle;
    private Translation3d translation;
    private PhotonCamera camera;
    private PhotonPoseEstimator poseEstimator;
    private AprilTagFieldLayout layout;

    public Camera(builder b){
        this.angle = b.angle;
        this.translation = b.translation;
        this.camera = b.camera;
        this.layout = b.layout;

        poseEstimator = new PhotonPoseEstimator(layout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, new Transform3d(translation, angle));   
    }
    public static class builder{
        private AprilTagFieldLayout layout;
        private Rotation3d angle;
        private Translation3d translation;
        private PhotonCamera camera;

        public builder(){
            layout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
            angle = new Rotation3d();
            translation = new Translation3d();
            camera = new PhotonCamera("");
        }

        public builder withAngle(Rotation3d rot){
            this.angle = rot;
            return this;
        }
        public builder withPosition(Translation3d pos){
            this.translation = pos;
            return this;
        }
        public builder withCamera(PhotonCamera cam){
            this.camera = cam;
            return this;
        }
        public builder withField(AprilTagFieldLayout layout){
            this.layout = layout;
            return this;
        }
        public Camera build(){
            return new Camera(this);
        }
    }
    
    public Transform3d getRobotToCam(){
        return new Transform3d(translation, angle);
    }
    public PhotonCamera getCamera(){
        return camera;
    }
    public EstimatedRobotPose update(){
        var out = poseEstimator.update();
        SmartDashboard.putBoolean(camera.getName()+" connected", camera.isConnected());
        if(out.isPresent()){
            return out.get();
        } else {
            return null;
        }
    }
}
