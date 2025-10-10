package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Arrays;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

import com.fasterxml.jackson.core.StreamWriteCapability;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.AffectorPosition;
import frc.robot.constants.Constants;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.WristConstants;
import frc.robot.subsystems.affector.Affector;
import frc.robot.subsystems.affector.Affector.WantedAffectorState;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.WantedIntakeState;
import frc.robot.subsystems.swerve.Drive;
import frc.robot.subsystems.swerve.Drive.WantedDriveState;
import frc.robot.subsystems.vision.Vision;
import frc.utils.AprilTagRegion;
import frc.utils.VariableLimSLR;

public class Superstructure extends SubsystemBase{
    public enum WantedSuperState {
        HOME,
        STOPPED,
        DEFAULT_STATE,
        INTAKE_CORAL,
        L4,
        L3,
        L2,
        L1,
        CLIMB
    }
    public enum CurrentSuperState {
        HOME,
        STOPPED,
        NO_PIECE_TELEOP,
        HOLDING_CORAL_TELEOP,
        NO_PIECE_AUTO,
        HOLDING_CORAL_AUTO,
        INTAKE_CORAL,
        L4,
        L3,
        L2,
        L1,
        CLIMB
    }
    // X = side to side, Y = away from tag
    public enum BranchSide{
        LEFT  (new Translation2d(Units.inchesToMeters(-14.75), Units.inchesToMeters(18))),
        RIGHT (new Translation2d(Units.inchesToMeters(-2)  , Units.inchesToMeters(17.5))),
        MIDDLE(new Translation2d());

        public Translation2d tagOffset;
        private BranchSide(Translation2d offsets) {
            tagOffset = offsets;
        }

        public BranchSide mirror(){
            switch (this) {
                case LEFT: return RIGHT;
                case MIDDLE: return MIDDLE;
                default: return LEFT;
            }
        }
    }
    public enum StationSide{
        LEFT  (DriverStation.getAlliance().get() == Alliance.Blue ? DriveConstants.presets.WEST_STATION : DriveConstants.presets.EAST_STATION),
        RIGHT (DriverStation.getAlliance().get() == Alliance.Blue ? DriveConstants.presets.EAST_STATION : DriveConstants.presets.WEST_STATION),
        AUTO  (new Pose2d());

        public Pose2d pos;
        private StationSide(Pose2d pos) {
            this.pos = pos;
        }

    }
    
    public WantedSuperState wantedState    =  WantedSuperState.DEFAULT_STATE;
    public CurrentSuperState currentState  = CurrentSuperState.STOPPED;
    public CurrentSuperState previousState = CurrentSuperState.STOPPED;

    private boolean affectorTransition = false;
    @AutoLogOutput(key="Superstructure/transitionWristThreshold")
    private double transitionWristThreshold = 45.0;
    private AffectorPosition bufferedPos = new AffectorPosition(0, 0);

    Intake intake;
    Climber climber;
    Affector affector;
    Drive drive;
    Vision vision;
    Led led;
    
    @AutoLogOutput(key="Superstructure/is scoring")
    private boolean scoring = false;
    
    private VariableLimSLR lxLim;
    private VariableLimSLR lyLim;
    private VariableLimSLR rxLim;
    private VariableLimSLR ryLim;
    
    private boolean fod = Constants.drive.STARTING_FOD;

    private Rotation2d stationAngle = new Rotation2d();

    private LoggedNetworkBoolean useVisionOdometry = new LoggedNetworkBoolean("overrides/useVisionOdometry", DriveConstants.USE_VISION);
    
    public static  final ArrayList<Pose2d> blueReefTagPoses = new ArrayList<>();
    public static  final ArrayList<Pose2d> redReefTagPoses = new ArrayList<>();
    public static  final ArrayList<Pose2d> allReefTagPoses = new ArrayList<>();

    private Pose2d branch = new Pose2d();//silly workaround for auto

    static{
        var field = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);

        Arrays.stream(AprilTagRegion.kReef.blue()).forEach((i) -> {
            field.getTagPose(i).ifPresent((p) -> {
                blueReefTagPoses.add(new Pose2d(
                    p.getMeasureX(),
                    p.getMeasureY(),
                    p.getRotation().toRotation2d()
                ));
            });
        });

        Arrays.stream(AprilTagRegion.kReef.red()).forEach((i) -> {
            field.getTagPose(i).ifPresent((p) -> {
                redReefTagPoses.add(new Pose2d(
                    p.getMeasureX(),
                    p.getMeasureY(),
                    p.getRotation().toRotation2d()
                ));
            });
        });

        Arrays.stream(AprilTagRegion.kReef.both()).forEach((i) -> {
            field.getTagPose(i).ifPresent((p) -> {
                allReefTagPoses.add(new Pose2d(
                    p.getMeasureX(),
                    p.getMeasureY(),
                    p.getRotation().toRotation2d()
                ));
            });
        });
    }

    public Superstructure(
            Drive drive, 
            Intake intake, 
            Climber climber, 
            Affector affector, 
            Vision vision, 
            Led led,
            VariableLimSLR lxLim,
            VariableLimSLR lyLim,
            VariableLimSLR rxLim,
            VariableLimSLR ryLim
        ){
        this.drive = drive;
        this.intake = intake;
        this.climber = climber;
        this.affector = affector;
        this.vision = vision;
        this.led = led;
        this.lxLim = lxLim;
        this.lyLim = lyLim;
        this.rxLim = rxLim;
        this.ryLim = ryLim;

    
    }

    @Override
    public void periodic() {
        previousState = currentState;

        
        double rlim = Double.POSITIVE_INFINITY;
        if(affector.getPosition().elev > .75){
          rlim = 1/0.2;
        }
        lxLim.setLim(rlim);
        lyLim.setLim(rlim);
        rxLim.setLim(rlim);
        ryLim.setLim(rlim);

        drive.setFOD(fod);

        // SmartDashboard.putBoolean("holding", !holdingSens.get());
        led.setRunning(intake.isMoving());

        led.hasCoral = intake.isHolding();

        led.homed = affector.isElevHomed();

        led.intakeSensorFault = !intake.getSensorEnabled();

        Logger.recordOutput("Drive/fieldOrientedDrive", getFOD());

        DriveConstants.USE_VISION = useVisionOdometry.get();

        if(DriverStation.getAlliance().isPresent()){
            if(DriverStation.getAlliance().get() == Alliance.Red){
                if(drive.getPose().getTranslation().getY() > 4){
                    stationAngle = DriveConstants.presets.EAST_STATION.getRotation();
                } else {
                    stationAngle = DriveConstants.presets.WEST_STATION.getRotation();
                }
            } else {
                if(drive.getPose().getTranslation().getY() > 4){
                    stationAngle = DriveConstants.presets.WEST_STATION.getRotation();
                } else {
                    stationAngle = DriveConstants.presets.EAST_STATION.getRotation();
                }
            }
        }


        updateAScopePoses();

        Logger.recordOutput("Superstructure/previousState", previousState);
        Logger.recordOutput("Superstructure/currentState", currentState);
        Logger.recordOutput("Superstructure/wantedState", wantedState);

        Logger.recordOutput("Superstructure/affector transition", affectorTransition);

        Logger.recordOutput("Superstructure/bufferedPos/elev" , bufferedPos.elev);
        Logger.recordOutput("Superstructure/bufferedPos/wrist", bufferedPos.wrist);
        
        stateTransition();
        applyStates();
        
    }

    //update current state
    private void stateTransition(){
        switch (wantedState) {
            case HOME:
                currentState = CurrentSuperState.HOME;
            break;
            case STOPPED:
                currentState = CurrentSuperState.STOPPED;
            break;
            case DEFAULT_STATE:
                
                if(intake.isHolding()){
                    currentState = CurrentSuperState.HOLDING_CORAL_TELEOP;
                } else {
                    currentState = CurrentSuperState.NO_PIECE_TELEOP;
                }
            break;
            case INTAKE_CORAL:
                currentState = CurrentSuperState.INTAKE_CORAL;
            break;
            case L4:
                currentState = CurrentSuperState.L4;
            break;
            case L3:
                currentState = CurrentSuperState.L3;
            break;
            case L2:
                currentState = CurrentSuperState.L2;
            break;
            case L1:
                currentState = CurrentSuperState.L1;
            break;
            case CLIMB:
                currentState = CurrentSuperState.CLIMB;
            break;
            default:
                currentState = CurrentSuperState.STOPPED;
            break;
        }
    }

    //apply current state
    private void applyStates(){
        if(scoring){
            if(!intake.isHolding() && intake.getSensorEnabled()){
                if(currentState != CurrentSuperState.L1){
                    endScore();
                }
                drive.setWantedState(WantedDriveState.TELEOP_DRIVE);
                CommandScheduler.getInstance().schedule(new InstantCommand(()->{}, drive));
            }
        }

        if(affectorTransition){
            if(affector.getPosition().wrist < Units.degreesToRadians(transitionWristThreshold)){
                affector.setWantedState(WantedAffectorState.POSITION, new AffectorPosition(affector.getPosition().elev, Constants.Affector.STOW_POSITION.wrist));
            } else {
                if(Math.abs(affector.getPosition().elev - bufferedPos.elev) < ElevatorConstants.NEAR_POS_TOLERANCE){
                    affector.setWantedState(WantedAffectorState.POSITION, bufferedPos);
                    affectorTransition = false;
                } else {
                    affector.setWantedState(WantedAffectorState.POSITION, new AffectorPosition(bufferedPos.elev, Constants.Affector.STOW_POSITION.wrist));
                }
            }
        }

        led.affectorInPos = affector.atSetpoint();

        if(currentState != CurrentSuperState.HOME){
            led.homing = false;
        }
        if(currentState != CurrentSuperState.CLIMB){
            led.climbMode = false;
        }

        switch(currentState){
            case HOME:
                if(previousState != CurrentSuperState.HOME){
                    affector.setWantedState(Affector.WantedAffectorState.HOME);
                }
                if(affector.isElevHomed()){
                    setWantedState(WantedSuperState.DEFAULT_STATE);
                }
                led.homing = true;
            break;
            case STOPPED:
            break;
            case NO_PIECE_TELEOP:
                if(affectorTransition){
                    bufferedPos = Constants.Affector.STOW_POSITION;
                }
                if(!scoring && currentState != previousState){
                    intake.setWantedState(WantedIntakeState.STOP);
                }
            break;
            case HOLDING_CORAL_TELEOP:
                if(affectorTransition){
                    bufferedPos = Constants.Affector.HOLD_POSITION;
                }
            break;
            case NO_PIECE_AUTO:
                if(affectorTransition){
                    bufferedPos = Constants.Affector.STOW_POSITION;
                }
                if(!scoring && currentState != previousState){
                    intake.setWantedState(WantedIntakeState.STOP);
                }
            break;
            case HOLDING_CORAL_AUTO:
            break;
            case INTAKE_CORAL:
                if(intake.isHolding() && intake.getSensorEnabled()){
                    setWantedState(WantedSuperState.DEFAULT_STATE);
                    intake.setWantedState(Intake.WantedIntakeState.STOP);
                }
            break;
            case L4:
                if(!intake.isHolding() && intake.getSensorEnabled()){
                    setWantedState(WantedSuperState.DEFAULT_STATE);
                }
            break;
            case L3:
                if(!intake.isHolding() && intake.getSensorEnabled()){
                    setWantedState(WantedSuperState.DEFAULT_STATE);
                }
            break;
            case L2:
                if(!intake.isHolding() && intake.getSensorEnabled()){
                    setWantedState(WantedSuperState.DEFAULT_STATE);
                }
            break;
            case L1:
                if(!intake.isHolding() && intake.getSensorEnabled() && !scoring){
                    setWantedState(WantedSuperState.DEFAULT_STATE);
                }
                if(affectorTransition){
                    bufferedPos = Constants.Affector.L1_POSITION;
                }
            break;
            case CLIMB:
                led.climbMode = true;
            break;
            default:
            break;
        }
    }

    public void setWantedState(WantedSuperState state){
        wantedState = state;

        if(currentState == CurrentSuperState.L2 || currentState == CurrentSuperState.L3){
            transitionWristThreshold = 70.0;
        } else {
            transitionWristThreshold = 45.0;
        }
        switch (state){
            case DEFAULT_STATE:
                affectorTransition = true;
                if(intake.getSensorEnabled()){
                    bufferedPos = intake.isHolding() ? Constants.Affector.HOLD_POSITION : Constants.Affector.STOW_POSITION;
                } else {
                    bufferedPos = Constants.Affector.STOW_POSITION;
                }
            break;
            case HOME:
                intake.setWantedState(Intake.WantedIntakeState.STOP);
            break;
            case STOPPED:
                intake.setWantedState(Intake.WantedIntakeState.STOP);
                affector.stop();
            break;
            case INTAKE_CORAL:
                if((!intake.isHolding() || !intake.getSensorEnabled()) && bufferedPos != Constants.Affector.STATION_POSITION){
                    affectorTransition = true;
                    bufferedPos = Constants.Affector.STATION_POSITION;
                    intake.setWantedState(Intake.WantedIntakeState.INTAKE);
                }
            break;
            case L1:
                if((intake.isHolding() || !intake.getSensorEnabled()) && bufferedPos != Constants.Affector.L1_POSITION){
                    affectorTransition = true;
                    bufferedPos = Constants.Affector.L1_POSITION;
                }
            break;
            case L2:
                if((intake.isHolding() || !intake.getSensorEnabled()) && bufferedPos != Constants.Affector.L2_POSITION){
                    affectorTransition = true;
                    bufferedPos = Constants.Affector.L2_POSITION;
                }
            break;
            case L3:
                if((intake.isHolding() || !intake.getSensorEnabled()) && bufferedPos != Constants.Affector.L3_POSITION){
                    affectorTransition = true;
                    bufferedPos = Constants.Affector.L3_POSITION;
                }
            break;
            case L4:
                if((intake.isHolding() || !intake.getSensorEnabled()) && bufferedPos != Constants.Affector.L4_POSITION){
                    affectorTransition = true;
                    bufferedPos = Constants.Affector.L4_POSITION;
                }
            break;
            case CLIMB:
                affector.setWantedState(Affector.WantedAffectorState.POSITION, Constants.Affector.STOW_POSITION);
            break;
            default:
            break;
        }
    }
    
  public boolean getFOD(){return fod;}
    
    /**
     * get closest reef april tag pose to given position
     * 
     * @param pose field relative position
     * @return
     */
    public static Pose2d getClosestReefAprilTag(Pose2d pose) {
        var alliance = DriverStation.getAlliance();
        
        ArrayList<Pose2d> reefPoseList;
        if (alliance.isEmpty()) {
            reefPoseList = allReefTagPoses;
        } else{
            reefPoseList = alliance.get() == Alliance.Blue ? 
                blueReefTagPoses :
                redReefTagPoses;
        }

        return pose.nearest(reefPoseList);
    }
    
  
    private static Pose2d getBranchFromTag(Pose2d tag, BranchSide side, boolean far) {
        var translation = tag.getTranslation().plus(
            new Translation2d(
                side.tagOffset.getY() + (far ? Units.inchesToMeters(5) : 0),
                side.tagOffset.getX()
            ).rotateBy(tag.getRotation())
        );    

        return new Pose2d(
            translation.getX(),
            translation.getY(),
            tag.getRotation().rotateBy(Rotation2d.kCCW_90deg)
        );
    }

    public void updateAScopePoses(){
        //actual pos
        Logger.recordOutput("AScope/componentPoses", new Pose3d[] {
            affector.calculatePoseElevMiddleStage(affector.getPosition().elev),
            affector.calculatePoseElevInnerStage(affector.getPosition().elev),
            affector.calculatePoseWrist(affector.getPosition().wrist, affector.getPosition().elev),
            intake.isHolding() ? new Pose3d(
                WristConstants.WRIST_POS.plus(new Translation3d(0, Math.cos(affector.getPosition().wrist)*IntakeConstants.pivotToCoral, affector.getPosition().elev + Math.sin(affector.getPosition().wrist)*IntakeConstants.pivotToCoral)),
                new Rotation3d(0, -affector.getPosition().wrist+Math.PI/2, 0).rotateBy(new Rotation3d(0, 0, Math.PI/2))
            ) : new Pose3d(new Translation3d(0, 0, -10), new Rotation3d()),
        });
        //setpoints
        Logger.recordOutput("AScope/componentSetPoses", new Pose3d[] {
            affector.calculatePoseElevMiddleStage(affector.getPositionSet().elev),
            affector.calculatePoseElevInnerStage(affector.getPositionSet().elev),
            affector.calculatePoseWrist(affector.getPositionSet().wrist, affector.getPositionSet().elev),
            new Pose3d(new Translation3d(0, 0, -10), new Rotation3d()),
        });
    }

    public boolean isReady(){
        return affector.isElevHomed()//elevator homed
            && affector.atSetpoint()//in pos
            && (isAffectorPosScoring(affector.getPositionSet()) ? intake.isHolding() || !intake.getSensorEnabled() : true)//holding if in scoring pos or bypass(hold lock override assumes sensor is non functional)
            && (affector.getPositionSet() == Constants.Affector.STATION_POSITION ? intake.isIntaking() : true);//intaking if in station pos
    }

    private boolean isAffectorPosScoring(AffectorPosition p){
        return p == Constants.Affector.L1_POSITION || p == Constants.Affector.L2_POSITION || p == Constants.Affector.L3_POSITION || p == Constants.Affector.L4_POSITION;
    }

    public void autoAlign(BranchSide side){

        Pose2d tag = getClosestReefAprilTag(drive.getPose());
        var branch = getBranchFromTag(tag, side, currentState == CurrentSuperState.L1);
        drive.setTargetPose(branch);
    }
    public Command getAutoAlign(BranchSide side){
        //TODO: align position is calculated at start of code, not on call
        return new InstantCommand(() -> {
            branch = getBranchFromTag(getClosestReefAprilTag(drive.getPose()), side, currentState == CurrentSuperState.L1);
        }).andThen(drive.getAutoAlign(() -> branch));
    }
    
    public void alignWithStation(StationSide side){
        Rotation2d angle = new Rotation2d(side.pos.getRotation().getRadians());

        if(DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red){
            angle = angle.rotateBy(Rotation2d.k180deg);
        }
        drive.setTargetRotation(angle.getRadians());
    }
    // public Command getStationAutoAlign(StationSide side){
    //     Pose2d p = side.pos.;

    //     if(DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red){
    //         p = p.rotateBy(Rotation2d.k180deg);
    //     }
    //     drive.setTargetRotation(p.getRadians());
    // }

    public void toggleFOD(){fod = !fod;}

    public void score(){
        scoring = true;
        
        if(currentState == CurrentSuperState.L1 || currentState == CurrentSuperState.INTAKE_CORAL){
            intake.setWantedState(WantedIntakeState.OUTTAKE);
        } else {
            intake.setWantedState(WantedIntakeState.SCORE);
        }
    }
    
    public void endScore(){
        scoring = false;
        intake.setWantedState(WantedIntakeState.STOP);
    }
}
