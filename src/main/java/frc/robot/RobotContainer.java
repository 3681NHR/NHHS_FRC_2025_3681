/* I wrote this robot code with furry paws on. Just thought I would mention that. -yarden*/

package frc.robot;

import frc.robot.commands.AnglePresetDriveCommand;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.HomeElevator;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.StationIntake;
import frc.robot.constants.AffectorPosition;
import frc.robot.constants.ClimberConstants;
import frc.robot.constants.Constants;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.Constants.OperatorConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.constants.WristConstants;
import frc.robot.subsystems.Led;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberIO;
import frc.robot.subsystems.climber.ClimberIOSpark;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.elevator.ElevatorIOSpark;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeIOSpark;
import frc.robot.subsystems.physButtons.ButtonIO;
import frc.robot.subsystems.physButtons.ButtonIODIO;
import frc.robot.subsystems.physButtons.ButtonIOSim;
import frc.robot.subsystems.physButtons.Buttons;
import frc.robot.subsystems.swerve.*;
import frc.robot.subsystems.vision.CameraIO;
import frc.robot.subsystems.vision.CameraIOPhoton;
import frc.robot.subsystems.vision.CameraIOPhotonSim;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.subsystems.wrist.WristIO;
import frc.robot.subsystems.wrist.WristIOSim;
import frc.robot.subsystems.wrist.WristIOSpark;
import frc.utils.rumble.*;
import frc.utils.TimerHandler;
import frc.utils.Joystick.duelJoystickAxis;
import frc.utils.BatteryVoltageSim;
import frc.utils.DisabledInstantCommand;
import frc.utils.ExtraMath;
import frc.utils.Joystick;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import static frc.utils.ControllerMap.*;

import java.util.function.DoubleSupplier;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggedPowerDistribution;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class RobotContainer {

  // Create and configure a drivetrain simulation configuration
  private DriveTrainSimulationConfig driveTrainSimulationConfig;
  private SwerveDriveSimulation driveSim;
  
  private Drive drive;
  private Vision vision;
  private Elevator elevator;
  private Wrist wrist;
  private Intake intake;
  private Buttons buttons;
  private Climber climber;

  private Led led = new Led();

  private final XboxController driverController =
      new XboxController(OperatorConstants.DRIVER_CONTROLLER_PORT);
      
  private final XboxController operatorController =
      new XboxController(OperatorConstants.OPERATOR_CONTROLLER_PORT);
  // private DigitalInput holdingSens = new DigitalInput(5);


  private LoggedNetworkBoolean resetOdometry = new LoggedNetworkBoolean("resetOdometry", false);
  private LoggedDashboardChooser<Command> autoChooser;
  private LoggedNetworkBoolean useVisionOdometry = new LoggedNetworkBoolean("overrides/useVisionOdometry", DriveConstants.USE_VISION);

  private boolean fod = Constants.drive.STARTING_FOD;
  private boolean directAngle = Constants.drive.STARTING_DIRECT_ANGLE;

  private int reefIndex = 0;
  private int farIndex = 0;
  private Rotation2d reefAngle = Constants.OperatorConstants.REEF_ROTS[0];
  private Rotation2d stationAngle = new Rotation2d();
  private Rotation2d farAngle = new Rotation2d();

  private DoubleSupplier lx;
  private DoubleSupplier ly;
  private DoubleSupplier rx;
  private DoubleSupplier ry;

  private duelJoystickAxis driverSticks;

  private DoubleSupplier leftTrigger;
  private DoubleSupplier rightTrigger;

  private RumbleHandler rumbler = new RumbleHandler(driverController);
  private RumbleHandler opRumbler = new RumbleHandler(operatorController);

  private PowerDistribution pdp = new PowerDistribution(1  , ModuleType.kRev);
  
  private final Alert driverDisconnected =
      new Alert("Driver controller disconnected (port 0).", AlertType.kWarning);
  private final Alert operatorDisconnected =
      new Alert("Operator controller disconnected (port 1).", AlertType.kWarning);


  @AutoLogOutput
  private AffectorPosition target = AffectorPosition.STOW;

  public RobotContainer() {

    Logger.recordOutput("zero", new Pose3d());

    DriverStation.silenceJoystickConnectionWarning(true);

    if(RobotBase.isSimulation()){
      driveTrainSimulationConfig = DriveTrainSimulationConfig.Default()
          .withGyro(COTS.ofPigeon2())
          .withSwerveModule(COTS.ofMark4i(
                  DCMotor.getNEO(1),
                  DCMotor.getNEO(1),
                  COTS.WHEELS.DEFAULT_NEOPRENE_TREAD.cof,
                  2))
          .withTrackLengthTrackWidth(Meters.of(DriveConstants.LENGTH), Meters.of(DriveConstants.WIDTH))
          .withBumperSize(Inches.of(30), Inches.of(27));

      driveSim = new SwerveDriveSimulation(driveTrainSimulationConfig, Constants.STARTING_POSE);
      // Register the drivetrain simulation to the default simulation world
      SimulatedArena.getInstance().addDriveTrainSimulation(driveSim);

      lx = () -> driverController.getRawAxis(LEFT_STICK_X);
      ly = () -> driverController.getRawAxis(LEFT_STICK_Y);
      rx = () -> driverController.getRawAxis(RIGHT_STICK_X);
      ry = () -> driverController.getRawAxis(RIGHT_STICK_Y);

      leftTrigger = () -> driverController.getRawAxis(LEFT_TRIGGER);
      rightTrigger = () -> driverController.getRawAxis(RIGHT_TRIGGER);
    } else {

      lx = () -> driverController.getLeftX();
      ly = () -> driverController.getLeftY();
      rx = () -> driverController.getRightX();
      ry = () -> driverController.getRightY();

      leftTrigger = () -> driverController.getLeftTriggerAxis();
      rightTrigger = () -> driverController.getRightTriggerAxis();


    }
    //process driver controls(radial deadzone, curve, trigger slowdown, and inversion)
    driverSticks = new duelJoystickAxis(
      () -> ExtraMath.processInput(Joystick.deadzone(Constants.OperatorConstants.LEFT_DEADBAND, lx.getAsDouble(), ly.getAsDouble()).getX()  , -ExtraMath.remap(leftTrigger.getAsDouble() , 0.0, 1.0, 1.0, 0.1), Constants.OperatorConstants.TRANSLATION_CURVE, 0.0),
      () -> ExtraMath.processInput(Joystick.deadzone(Constants.OperatorConstants.LEFT_DEADBAND, lx.getAsDouble(), ly.getAsDouble()).getY()  , -ExtraMath.remap(leftTrigger.getAsDouble() , 0.0, 1.0, 1.0, 0.1), Constants.OperatorConstants.TRANSLATION_CURVE, 0.0),
      () -> ExtraMath.processInput(Joystick.deadzone(Constants.OperatorConstants.RIGHT_DEADBAND, rx.getAsDouble(), ry.getAsDouble()).getX(), -ExtraMath.remap(rightTrigger.getAsDouble(), 0.0, 1.0, 1.0, 0.1), Constants.OperatorConstants.ROTATION_CURVE   , 0.0),
      () -> ExtraMath.processInput(Joystick.deadzone(Constants.OperatorConstants.RIGHT_DEADBAND, rx.getAsDouble(), ry.getAsDouble()).getY(), -ExtraMath.remap(rightTrigger.getAsDouble(), 0.0, 1.0, 1.0, 0.1), Constants.OperatorConstants.ROTATION_CURVE   , 0.0)
    );

    switch (Constants.MODE) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        vision = new Vision(
          new CameraIOPhoton(VisionConstants.CAMERA_NAMES[0], VisionConstants.CAMERA_0_ROBOT_TO_CAM),
          new CameraIOPhoton(VisionConstants.CAMERA_NAMES[1], VisionConstants.CAMERA_1_ROBOT_TO_CAM));
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOSpark(0),
                new ModuleIOSpark(1),
                new ModuleIOSpark(2),
                new ModuleIOSpark(3),
                vision);
        elevator = new Elevator(new ElevatorIOSpark());
        wrist = new Wrist(new WristIOSpark());
        intake = new Intake(new IntakeIOSpark());
        buttons = new Buttons(new ButtonIODIO(4));
        climber = new Climber(new ClimberIOSpark());
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        vision = new Vision(
          new CameraIOPhotonSim(VisionConstants.CAMERA_NAMES[0], VisionConstants.CAMERA_0_ROBOT_TO_CAM, driveSim::getSimulatedDriveTrainPose),
          new CameraIOPhotonSim(VisionConstants.CAMERA_NAMES[1], VisionConstants.CAMERA_1_ROBOT_TO_CAM, driveSim::getSimulatedDriveTrainPose)
          );
        if(driveSim != null){
          drive =
              new Drive(
                  new GyroIOSim(driveSim.getGyroSimulation()) {},
                  new ModuleIOSim(driveSim.getModules()[0]),
                  new ModuleIOSim(driveSim.getModules()[1]),
                  new ModuleIOSim(driveSim.getModules()[2]),
                  new ModuleIOSim(driveSim.getModules()[3]),
                  vision);
        elevator = new Elevator(new ElevatorIOSim());
        elevator.setHomed(true);
        wrist = new Wrist(new WristIOSim());
        intake = new Intake(new IntakeIOSim(driveSim, elevator, wrist));
        buttons = new Buttons(new ButtonIOSim(() -> false));
        climber = new Climber(new ClimberIO() {});
        }
        break;

      default:
        // Replayed robot, disable IO implementations
        vision = new Vision(
          new CameraIO() {},
          new CameraIO() {}
        );
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                vision);
        elevator = new Elevator(new ElevatorIO() {});
        wrist = new Wrist(new WristIO() {});
        intake = new Intake(new IntakeIO() {});
        buttons = new Buttons(new ButtonIO() {});
        climber = new Climber(new ClimberIO() {});
        break;
    }

    
    NamedCommands.registerCommand("station", new StationIntake(elevator, wrist, intake));
    NamedCommands.registerCommand("L2", Commands.runOnce(() -> {
        elevator.setTargetPos(AffectorPosition.L2.elev);
        wrist.setPosSet(AffectorPosition.L2.wrist);
      }, elevator, wrist, intake));
    NamedCommands.registerCommand("L3", Commands.runOnce(() -> {
      elevator.setTargetPos(AffectorPosition.L3.elev);
      wrist.setPosSet(AffectorPosition.L3.wrist);
    }, elevator, wrist, intake));
  NamedCommands.registerCommand("L4", Commands.runOnce(() -> {
    elevator.setTargetPos(AffectorPosition.L4.elev);
    wrist.setPosSet(AffectorPosition.L4.wrist);
  }, elevator, wrist, intake));
  NamedCommands.registerCommand("stow", Commands.runOnce(() -> {
    elevator.setTargetPos(AffectorPosition.STOW.elev);
    wrist.setPosSet(AffectorPosition.STOW.wrist);
  }, elevator, wrist, intake));

NamedCommands.registerCommand("score", Commands.run(() -> intake.setVoltage(IntakeConstants.SPEED),intake).finallyDo(() -> intake.stop()).until(() -> !intake.isHolding()));


    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    if(DriverStation.isTest()){
      // Set up SysId routines
      autoChooser.addOption(
          "Drive SysId (Quasistatic Forward/Reverse)",
          drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward).andThen(drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse)));
      autoChooser.addOption(
          "Drive SysId (Dynamic Forward/Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward).andThen(drive.sysIdDynamic(SysIdRoutine.Direction.kReverse)));
      autoChooser.addOption(
          "Steer SysId (Quasistatic Forward/Reverse)",
          drive.steerSysIdQuasistatic(SysIdRoutine.Direction.kForward).andThen(drive.steerSysIdQuasistatic(SysIdRoutine.Direction.kReverse)));
      autoChooser.addOption(
          "Steer SysId (Dynamic Forward/Reverse)", drive.steerSysIdDynamic(SysIdRoutine.Direction.kForward).andThen(drive.steerSysIdDynamic(SysIdRoutine.Direction.kReverse)));
      autoChooser.addOption(
          "Angle SysId (Quasistatic Forward/Reverse)",
          drive.angleSysIdQuasistatic(SysIdRoutine.Direction.kForward).andThen(drive.angleSysIdQuasistatic(SysIdRoutine.Direction.kReverse)));
      autoChooser.addOption(
          "Angle SysId (Dynamic Forward/Reverse)", drive.angleSysIdDynamic(SysIdRoutine.Direction.kForward).andThen(drive.angleSysIdDynamic(SysIdRoutine.Direction.kReverse)));
      autoChooser.addOption(
        "Elevator SysId (Quasistatic Forward)", elevator.sysIdQuasistatic(SysIdRoutine.Direction.kForward).withName("sysid"));
      autoChooser.addOption(
        "Elevator SysId (Quasistatic Reverse)", elevator.sysIdQuasistatic(SysIdRoutine.Direction.kReverse).withName("sysid"));
      autoChooser.addOption(
        "Elevator SysId (Dynamic Forward)", elevator.sysIdDynamic(SysIdRoutine.Direction.kForward).withName("sysid"));
      autoChooser.addOption(
        "Elevator SysId (Dynamic Reverse)", elevator.sysIdDynamic(SysIdRoutine.Direction.kReverse).withName("sysid"));
      autoChooser.addOption(
        "Wrist SysId (Quasistatic Forward)", wrist.sysIdQuasistatic(SysIdRoutine.Direction.kForward).withName("sysid"));
      autoChooser.addOption(
        "Wrist SysId (Quasistatic Reverse)", wrist.sysIdQuasistatic(SysIdRoutine.Direction.kReverse).withName("sysid"));
      autoChooser.addOption(
        "Wrist SysId (Dynamic Forward)", wrist.sysIdDynamic(SysIdRoutine.Direction.kForward).withName("sysid"));
      autoChooser.addOption(
        "Wrist SysId (Dynamic Reverse)", wrist.sysIdDynamic(SysIdRoutine.Direction.kReverse).withName("sysid"));
      } 

    configureBindings();

    LoggedPowerDistribution.getInstance(pdp.getModule(), ModuleType.kRev);
    
    // left stick controls translation
    // right stick controls the angular velocity of the robot
    // sim command used raw axis for simulating joysticks
    Command driveCommand = DriveCommands.driveCommand(
      driverSticks,  
      () -> this.getDirectAngle(),
      () -> this.getFOD(),
      drive
    );

    drive.setDefaultCommand(driveCommand);
    wrist.setDefaultCommand(wrist.man(() -> ExtraMath.processInput(operatorController.getRightY(), -0.02 * WristConstants.POS_PID.maxSpeed(), 1.0, 0.05)));
    elevator.setDefaultCommand(elevator.man(() -> (operatorController.getRightTriggerAxis()-operatorController.getLeftTriggerAxis())*OperatorConstants.ELEVATOR_MAN_SENS));
  }

  private void configureBindings() {
    resetOdometry.set(false);
    new Trigger(() ->resetOdometry.get()).onTrue(new InstantCommand(() -> {
      resetOdometry.set(false);
      drive.setPose(Constants.STARTING_POSE);
    }));
    
    
    //lock in place
    new Trigger(() -> driverController.getRawButton(LOGO_RIGHT)).whileTrue(new InstantCommand(() -> {
      drive.stopWithX();
      rumbler.overrideQue(new Rumble(.1, 0.25));
    }, drive).repeatedly());

    //reset angle
    new Trigger(() -> driverController.getRawButton(LOGO_LEFT)).onTrue(new InstantCommand(() -> {
      drive.resetGyro(DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red ? Math.PI : 0);
      rumbler.overrideQue(RumblePreset.TAP.load());
    }));

    //fod toggle
    new Trigger(() -> driverController.getRawButton(LEFT_STICK_BUTTON)).onTrue(new InstantCommand(() -> {
    this.fod = !this.fod;
    }));
    //DA toggle
    new Trigger(() -> driverController.getRawButton(RIGHT_STICK_BUTTON)).onTrue(new InstantCommand(() -> {
      this.directAngle = !this.directAngle;
    }));
    //timer alert
    new Trigger(() -> TimerHandler.getTeleopRemaining()<Constants.ENDGAME_TIME).onTrue(new InstantCommand(() -> {
      rumbler.overrideQue(RumblePreset.DOUBLE_TAP.load());
      opRumbler.overrideQue(RumblePreset.DOUBLE_TAP.load());
    }));
    //aim to station
    new Trigger(() -> driverController.getRawButton(B)).onTrue(new AnglePresetDriveCommand(
      driverSticks,
      drive,
      () -> stationAngle
    ));
    //aim for reef
    new Trigger(() -> driverController.getRawButton(Y)).onTrue(new AnglePresetDriveCommand(driverSticks, drive, () -> reefAngle));
    //aim for far elemant(prossesor or barge)
    new Trigger(() -> driverController.getPOV() == 0).onTrue(new InstantCommand(() -> {
      farIndex++;
      farIndex = farIndex % 2;
    }).andThen(new AnglePresetDriveCommand(driverSticks, drive, () -> farAngle)));

    //change target reef
    new Trigger(() -> driverController.getPOV() == 90).or(() -> operatorController.getRawButton(LOGO_RIGHT)).onTrue(new InstantCommand(() -> {
      reefIndex++;
      reefIndex = reefIndex % Constants.OperatorConstants.REEF_ROTS.length;
    }));
    new Trigger(() -> driverController.getPOV() == 270).or(() -> operatorController.getRawButton(LOGO_LEFT)).onTrue(new InstantCommand(() -> {
      reefIndex--;
      reefIndex = reefIndex % Constants.OperatorConstants.REEF_ROTS.length;
      if(reefIndex < 0){
        reefIndex = Constants.OperatorConstants.REEF_ROTS.length-1;
      }
    }));
 
    //physical button
    new Trigger(() -> buttons.get(0)).onTrue(new DisabledInstantCommand(() -> {
      if(DriverStation.isDisabled()){
        //elevator.toggleBrake();
        //wrist.toggleBrake();
      }
    })).debounce(1).onFalse(new DisabledInstantCommand(() -> {
      elevator.resetPos(ElevatorConstants.HOME_POS);
      elevator.setHomed(true);
    }));

    //home
    new Trigger(() -> operatorController.getRawButton(A)).onTrue(new HomeElevator(elevator));
        
    //intake controls
    new Trigger(() -> driverController.getRawButton(RB)).or(() -> operatorController.getRawButton(RB)).whileTrue(new IntakeCommand(intake));
    new Trigger(() -> operatorController.getRawButton(Y)).onTrue(new InstantCommand(() -> {
      intake.setVoltage(-IntakeConstants.SPEED);
    })).onFalse(new InstantCommand(() -> {
      intake.stop(); 
    }));

    //set affector target
    new Trigger(() -> operatorController.getPOV() == 180).onTrue(new InstantCommand(() -> {target = AffectorPosition.L1;}));
    new Trigger(() -> operatorController.getPOV() == 90).onTrue(new InstantCommand(() -> {target = AffectorPosition.STATION;}));
    new Trigger(() -> operatorController.getPOV() == 270).onTrue(new InstantCommand(() -> {target = target == AffectorPosition.L2 ? AffectorPosition.L3 : AffectorPosition.L2;}));
    new Trigger(() -> operatorController.getPOV() == 0).onTrue(new InstantCommand(() -> {target = AffectorPosition.L4;}));

    //go to affector target
    new Trigger(() -> driverController.getRawButton(LB)).or(() -> operatorController.getRawButton(LB))
      .onTrue(new InstantCommand(() -> {
        elevator.setTargetPos(target.elev);
        wrist.setPosSet(target.wrist);
      }));

    new Trigger(() -> driverController.getRawButton(X))
    .and(() -> ExtraMath.getDistance(drive.getPose(), ExtraMath.getNearestPose(DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue ? Constants.positions.REEFS : Constants.positions.RED_REEFS, drive.getPose())) < .5)
    .whileTrue(Commands.run(() -> {
      drive.driveToPose(ExtraMath.getNearestPose(DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue ? Constants.positions.REEFS : Constants.positions.RED_REEFS, drive.getPose()));
    }));

    //climber
    new Trigger(() -> operatorController.getRawButton(X))
      .onTrue(new InstantCommand(() -> climber.setVoltage(ClimberConstants.EXTEND_VOLTAGE)))
      .onFalse(new InstantCommand(() -> climber.setVoltage(0)));
    
    new Trigger(() -> operatorController.getRawButton(B))
      .onTrue(new InstantCommand(() -> climber.setVoltage(ClimberConstants.RETRACT_VOLTAGE)))
      .onFalse(new InstantCommand(() -> climber.setVoltage(0)));
  }


  public void Periodic(){
    // SmartDashboard.putBoolean("holding", !holdingSens.get());
    led.setHomed(elevator.isHomed());
    led.setIntaking(intake.isMoving());
    
    led.setColor(isReady() ? Color.kWhite : intake.isHolding() ? Color.kGreen : Color.kOrange);

    Logger.recordOutput("fieldOrientedDrive", getFOD());
    Logger.recordOutput("directAngle", getDirectAngle());

    DriveConstants.USE_VISION = useVisionOdometry.get();

    rumbler.update(0.02);

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
    reefAngle = Constants.OperatorConstants.REEF_ROTS[reefIndex];
    Logger.recordOutput("reefindex", reefIndex);
    if(farIndex == 1){
      farAngle = DriveConstants.presets.PROSSESOR.getRotation();
    } else {
      farAngle = DriveConstants.presets.CLIMB;
    }
    Logger.recordOutput("farIndex", farIndex);

    driverDisconnected.set(!driverController.isConnected());
    operatorDisconnected.set(!operatorController.isConnected());

    updateAScopePoses();
  }

  public void SimPeriodic(){
    Logger.recordOutput("simulatedVoltage", BatteryVoltageSim.getInstance().calculateVoltage());

    SimulatedArena.getInstance().simulationPeriodic();

    Logger.recordOutput("FieldSimulation/RobotPose", driveSim.getSimulatedDriveTrainPose());

    Logger.recordOutput("FieldSimulation/Algae", 
      SimulatedArena.getInstance().getGamePiecesArrayByType("Algae"));
    Logger.recordOutput("FieldSimulation/Coral", 
      SimulatedArena.getInstance().getGamePiecesArrayByType("Coral"));

  }

  public Command getAutonomousCommand() {
    Command auto = autoChooser.get();
    return auto;
  }

  public void resetDrivetrain(Pose2d pose){
    driveSim.setSimulationWorldPose(pose);
  }
  
  public boolean getFOD(){return fod;}
  public boolean getDirectAngle(){return directAngle;}

  public void enable(){
    elevator.setBrake(true);
    wrist.setBrake(true);
  }


  public void updateAScopePoses(){
    //actual pos
    Logger.recordOutput("componentPoses", new Pose3d[] {
        elevator.getAScopePoseMiddleStage(elevator.getPositionSet()),
        elevator.getAScopePoseInnerStage(elevator.getPositionSet()),
        wrist.getAScopePoseWrist(wrist.getPosSet(), elevator.getPosition()),
        intake.isHolding() ? new Pose3d(
            WristConstants.WRIST_POS.plus(new Translation3d(0, Math.cos(wrist.getPos())*IntakeConstants.pivotToCoral, elevator.getPosition() + Math.sin(wrist.getPos())*IntakeConstants.pivotToCoral)),
            new Rotation3d(0, -wrist.getPos()+Math.PI/2, 0).rotateBy(new Rotation3d(0, 0, Math.PI/2))
        ) : new Pose3d(new Translation3d(0, 0, -10), new Rotation3d()),
    });
    //setpoints
    Logger.recordOutput("componentSetPoses", new Pose3d[] {
      elevator.getAScopePoseMiddleStage(elevator.getPositionSet()),
      elevator.getAScopePoseInnerStage(elevator.getPositionSet()),
      wrist.getAScopePoseWrist(wrist.getPosSet(), elevator.getPositionSet()),
      new Pose3d(new Translation3d(0, 0, -10), new Rotation3d()),
    });
    //targets, may not be applied
    Logger.recordOutput("componentTargetPoses", new Pose3d[] {
      elevator.getAScopePoseMiddleStage(target.elev),
      elevator.getAScopePoseInnerStage(target.elev),
      wrist.getAScopePoseWrist(target.wrist, target.elev),
      new Pose3d(new Translation3d(0, 0, -10), new Rotation3d()),
    });
  }
  public void updateLEDs(){
    led.setColor(
        isReady() ? Color.kWhite
        : intake.isHolding() ? Color.kGreen : Color.kOrange
    );
    led.setHomed(elevator.isHomed());
    led.setIntaking(intake.isMoving());
  }

  public boolean isReady(){
    return elevator.isHomed()//elevator homed
     && elevator.inPosition() && wrist.inPosition()//in pos
     && (target.isScoring() ? intake.isHolding() || !intake.getHoldLock() : true)//holding if in scoring pos
     && (target == AffectorPosition.STATION ? intake.isIntaking() : true)//intaking if in station pos
     && elevator.getPositionSet() == target.elev
     && wrist.getPosSet() == target.wrist;
  }
}
