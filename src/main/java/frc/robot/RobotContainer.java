package frc.robot;

import frc.robot.commands.AnglePresetDriveCommand;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.HomeElevator;
import frc.robot.constants.Constants;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.Constants.OperatorConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.elevator.ElevatorIOSpark;
import frc.robot.subsystems.swerve.*;
import frc.robot.subsystems.vision.CameraIO;
import frc.robot.subsystems.vision.CameraIOPhoton;
import frc.robot.subsystems.vision.CameraIOPhotonSim;
import frc.robot.subsystems.vision.Vision;
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
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggedPowerDistribution;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;
import com.pathplanner.lib.auto.AutoBuilder;

import choreo.auto.AutoFactory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
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

  private final XboxController driverController =
      new XboxController(OperatorConstants.DRIVER_CONTROLLER_PORT);
      
  private final XboxController operatorController =
      new XboxController(OperatorConstants.OPERATOR_CONTROLLER_PORT);

  private LoggedNetworkBoolean resetOdometry = new LoggedNetworkBoolean("resetOdometry", false);
  private LoggedDashboardChooser<Command> autoChooser;
  private LoggedNetworkBoolean useVisionOdometry = new LoggedNetworkBoolean("overrides/useVisionOdometry", DriveConstants.USE_VISION);

  private boolean fod = Constants.drive.STARTING_FOD;
  private boolean directAngle = Constants.drive.STARTING_DIRECT_ANGLE;

  private Trigger lockPose;
  private Trigger rstGyro;
  private Trigger toggleFOD;
  private Trigger toggleDA;

  private Trigger autoAimReef;
  private Trigger autoAimStation;
  private Trigger autoAimFar;//barge or prosseser

  private Trigger reefAimUp;
  private Trigger reefAimDown;

  private Trigger toggleElevBrake;

  private int reefIndex = 0;
  private int stationIndex = 0;
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

  private DigitalInput brakeDio = new DigitalInput(2);

  private RumbleHandler rumbler = new RumbleHandler(driverController);

  private PowerDistribution pdp = new PowerDistribution(1  , ModuleType.kRev);
  
  private final Alert driverDisconnected =
      new Alert("Driver controller disconnected (port 0).", AlertType.kWarning);
  private final Alert operatorDisconnected =
      new Alert("Operator controller disconnected (port 1).", AlertType.kWarning);

  public RobotContainer() {

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
          new CameraIOPhoton(VisionConstants.CAMERA_0_NAME, VisionConstants.CAMERA_0_ROBOT_TO_CAM),
          new CameraIOPhoton(VisionConstants.CAMERA_1_NAME, VisionConstants.CAMERA_1_ROBOT_TO_CAM));
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOSpark(0),
                new ModuleIOSpark(1),
                new ModuleIOSpark(2),
                new ModuleIOSpark(3),
                vision);
        elevator = new Elevator(new ElevatorIOSpark());
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        vision = new Vision(
          new CameraIOPhotonSim(VisionConstants.CAMERA_0_NAME, VisionConstants.CAMERA_0_ROBOT_TO_CAM, driveSim::getSimulatedDriveTrainPose),
          new CameraIOPhotonSim(VisionConstants.CAMERA_1_NAME, VisionConstants.CAMERA_1_ROBOT_TO_CAM, driveSim::getSimulatedDriveTrainPose)
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
        break;
    }

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
    
    elevator.setDefaultCommand(elevator.man(() -> (operatorController.getRightTriggerAxis()-operatorController.getLeftTriggerAxis())*OperatorConstants.ELEVATOR_MAN_SENS));
  }

  private void configureBindings() {
    resetOdometry.set(false);
    new Trigger(() ->resetOdometry.get()).onTrue(Commands.runOnce(() -> {
      resetOdometry.set(false);
      drive.setPose(Constants.STARTING_POSE);
    }));

    if(RobotBase.isReal()){
      lockPose = new Trigger(driverController::getStartButton);
      rstGyro = new Trigger(driverController::getBackButton);
      autoAimReef = new Trigger(driverController::getYButton);
      toggleFOD = new Trigger(driverController::getLeftStickButton);
      toggleDA = new Trigger(driverController::getRightStickButton);
      autoAimStation = new Trigger(driverController::getBButton);

    } else {
      lockPose = new Trigger(() -> driverController.getRawButton(LOGO_RIGHT));
      rstGyro = new Trigger(() -> driverController.getRawButton(LOGO_LEFT));
      autoAimReef = new Trigger(() -> driverController.getRawButton(Y));
      toggleFOD = new Trigger(() -> driverController.getRawButton(LEFT_STICK_BUTTON));
      toggleDA = new Trigger(() -> driverController.getRawButton(RIGHT_STICK_BUTTON));
      autoAimStation = new Trigger(() -> driverController.getRawButton(B));
    }
    autoAimFar = new Trigger(() -> driverController.getPOV() == 0);
    
    reefAimUp = new Trigger(() -> driverController.getPOV() == 90);
    reefAimDown = new Trigger(() -> driverController.getPOV() == 270);

    toggleElevBrake = new Trigger(() -> brakeDio.get());
      
    lockPose.whileTrue(Commands.runOnce(() -> {
      drive.stopWithX();
      rumbler.overrideQue(new Rumble(.1, 0.25));
    }, drive).repeatedly());

    rstGyro.onTrue(Commands.runOnce(() -> {
      drive.resetGyro(DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red ? Math.PI : 0);
      rumbler.overrideQue(RumblePreset.TAP.load());
    }));

    toggleFOD.onTrue(Commands.runOnce(() -> {
    this.fod = !this.fod;
    }));

    toggleDA.onTrue(Commands.runOnce(() -> {
      this.directAngle = !this.directAngle;
    }));

    new Trigger(() -> TimerHandler.getTeleopRemaining()<Constants.ENDGAME_TIME).onTrue(Commands.runOnce(() -> {
      rumbler.overrideQue(RumblePreset.DOUBLE_TAP.load());
    }));
    
    autoAimStation.onTrue(new AnglePresetDriveCommand(
      driverSticks,
      drive,
      () -> stationAngle
    ));

    autoAimReef.onTrue(new AnglePresetDriveCommand(driverSticks, drive, () -> reefAngle));
    
    autoAimFar.onTrue(Commands.runOnce(() -> {
      farIndex++;
      farIndex = farIndex % 2;
    }));
    autoAimFar.onTrue(new AnglePresetDriveCommand(driverSticks, drive, () -> farAngle));

    reefAimUp.onTrue(Commands.runOnce(() -> {
      reefIndex++;
      reefIndex = reefIndex % Constants.OperatorConstants.REEF_ROTS.length;
    }));
    reefAimDown.onTrue(Commands.runOnce(() -> {
      reefIndex--;
      reefIndex = reefIndex % Constants.OperatorConstants.REEF_ROTS.length;
      if(reefIndex < 0){
        reefIndex = Constants.OperatorConstants.REEF_ROTS.length-1;
      }
    }));

    toggleElevBrake.onTrue(new DisabledInstantCommand(() -> {
      elevator.toggleBrake();
    }));

    new Trigger(() -> operatorController.getAButton()).onTrue(new HomeElevator(elevator));
    
  }

  public void Periodic(){
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

}
