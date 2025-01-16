package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Camera {
    private Rotation3d angle;
    private Translation3d translation;
    private PhotonCamera camera;
    private PhotonPoseEstimator poseEstimator;
    private AprilTagFieldLayout layout;

    private PhotonCameraSim simCamera;
    private VisionSystemSim visionSystemSim;
    private SimCameraProperties simprop = new SimCameraProperties();

    public Camera(builder b){
        this.angle = b.angle;
        this.translation = b.translation;
        this.camera = b.camera;
        this.layout = b.layout;

        if(RobotBase.isSimulation()){
            simprop.setFPS(50);
            simprop.setAvgLatencyMs(10);
            visionSystemSim = new VisionSystemSim("sim vision");
            simCamera = new PhotonCameraSim(camera, simprop);

            simCamera.enableProcessedStream(true);
            simCamera.enableDrawWireframe(true);

            visionSystemSim.addAprilTags(layout);
            visionSystemSim.addCamera(simCamera, getRobotToCam());
        }

        poseEstimator = new PhotonPoseEstimator(layout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, new Transform3d(translation, angle));   
    }
    public static class builder{
        private AprilTagFieldLayout layout;
        private Rotation3d angle;
        private Translation3d translation;
        private PhotonCamera camera;

        public builder(){
            layout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
            angle = new Rotation3d();
            translation = new Translation3d();
            camera = new PhotonCamera("camera");
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
        Optional<EstimatedRobotPose> out = Optional.empty();
        for(PhotonPipelineResult p : camera.getAllUnreadResults()){
            out = poseEstimator.update(p);
        }
        SmartDashboard.putBoolean(camera.getName()+" connected", camera.isConnected());
        if(out.isPresent()){
            return out.get();
        } else {
            return null;
        }
    }
    public EstimatedRobotPose updateSim(Pose3d robotPose){
        SmartDashboard.putBoolean(camera.getName()+" connected(sim)", camera.isConnected());
        visionSystemSim.update(robotPose);

        Optional<EstimatedRobotPose> out = Optional.empty();
        for(PhotonPipelineResult p : camera.getAllUnreadResults()){
            out = poseEstimator.update(p);
        }
        if(out.isPresent()){
            return out.get();
        } else {
            return null;
        }
    }
}
