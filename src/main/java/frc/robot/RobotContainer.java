package frc.robot;

import frc.robot.commands.AnglePresetDriveCommand;
import frc.robot.commands.DriveCommands;
import frc.robot.constants.Constants;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.Constants.OperatorConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.swerve.*;
import frc.robot.subsystems.vision.CameraIO;
import frc.robot.subsystems.vision.CameraIOPhoton;
import frc.robot.subsystems.vision.CameraIOPhotonSim;
import frc.robot.subsystems.vision.Vision;
import frc.utils.rumble.*;
import frc.utils.TimerHandler;
import frc.utils.BatteryVoltageSim;
import frc.utils.ExtraMath;
import frc.utils.Joystick;

import static edu.wpi.first.units.Units.Inches;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggedPowerDistribution;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

@SuppressWarnings("unused")
public class RobotContainer {

  // Create and configure a drivetrain simulation configuration
  private DriveTrainSimulationConfig driveTrainSimulationConfig;
  private SwerveDriveSimulation driveSim;

  private Drive drive;

  private LoggedNetworkBoolean resetOdometry = new LoggedNetworkBoolean("resetOdometry", false);

  private LoggedDashboardChooser<Command> autoChooser;

  private boolean fod = Constants.drive.STARTING_FOD;
  private boolean directAngle = Constants.drive.STARTING_DIRECT_ANGLE;

  private Trigger lockPose;
  private Trigger rstGyro;

  private final XboxController driverController =
      new XboxController(OperatorConstants.DRIVER_CONTROLLER_PORT);

  private DoubleSupplier lx = driverController::getLeftX;
  private DoubleSupplier ly = driverController::getLeftY;
  private DoubleSupplier rx = driverController::getRightX;
  private DoubleSupplier ry = driverController::getRightY;

  private DoubleSupplier leftTrigger = driverController::getLeftTriggerAxis;
  private DoubleSupplier rightTrigger = driverController::getRightTriggerAxis;

  private RumbleHandler rumbler = new RumbleHandler(driverController);

  private PowerDistribution pdp = new PowerDistribution(1  , ModuleType.kRev);

  private Rotation2d povDRot = new Rotation2d();
  private Rotation2d povRRot = new Rotation2d();
  private Rotation2d povLRot = new Rotation2d();
  private Rotation2d povURot = new Rotation2d();

  public RobotContainer() {

    if(RobotBase.isSimulation()){
      driveTrainSimulationConfig = DriveTrainSimulationConfig.Default()
      // Specify gyro type (for realistic gyro drifting and error simulation)
      .withGyro(COTS.ofPigeon2())
      // Specify swerve module (for realistic swerve dynamics)
      .withSwerveModule(COTS.ofMark4(
              DCMotor.getNEO(1), // Drive motor is a Kraken X60
              DCMotor.getNEO(1), // Steer motor is a Falcon 500
              COTS.WHEELS.COLSONS.cof, // Use the COF for Colson Wheels
              2)) // L3 Gear ratio
      // Configures the track length and track width (spacing between swerve modules)
      .withTrackLengthTrackWidth(Inches.of(28), Inches.of(25.75))
      // Configures the bumper size (dimensions of the robot bumper)
      .withBumperSize(Inches.of(32), Inches.of(32));
      driveSim = new SwerveDriveSimulation(driveTrainSimulationConfig, Constants.STARTING_POSE);
      // Register the drivetrain simulation to the default simulation world
      SimulatedArena.getInstance().addDriveTrainSimulation(driveSim);

      lx = () -> driverController.getRawAxis(0);
      ly = () -> driverController.getRawAxis(1);
      rx = () -> driverController.getRawAxis(4);
      ry = () -> driverController.getRawAxis(5);

      leftTrigger = () -> driverController.getRawAxis(2);
      rightTrigger = () -> driverController.getRawAxis(3);
    }
    

    switch (Constants.MODE) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOSpark(0),
                new ModuleIOSpark(1),
                new ModuleIOSpark(2),
                new ModuleIOSpark(3),
                new Vision(
                  new CameraIOPhoton(VisionConstants.CAMERA_0_NAME, VisionConstants.CAMERA_0_ROBOT_TO_CAM)
                )
            );
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        if(driveSim != null){
          drive =
              new Drive(
                  new GyroIOSim(driveSim.getGyroSimulation()) {},
                  new ModuleIOSim(driveSim.getModules()[0]),
                  new ModuleIOSim(driveSim.getModules()[1]),
                  new ModuleIOSim(driveSim.getModules()[2]),
                  new ModuleIOSim(driveSim.getModules()[3]),
                  new Vision(
                  new CameraIOPhotonSim(VisionConstants.CAMERA_0_NAME, VisionConstants.CAMERA_0_ROBOT_TO_CAM, drive::getPose)
                ));
        }
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new Vision(new CameraIO() {}));
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
    }

    configureBindings();

    LoggedPowerDistribution.getInstance(pdp.getModule(), ModuleType.kRev);
    
    // left stick controls translation
    // right stick controls the angular velocity of the robot
    // sim command used raw axis for simulating joysticks
    Command driveCommand = DriveCommands.driveCommand(
      () -> ExtraMath.processInput(Joystick.deadzone(Constants.OperatorConstants.LEFT_DEADBAND, lx.getAsDouble(), ly.getAsDouble()).getX()  , -ExtraMath.remap(leftTrigger.getAsDouble() , 0.0, 1.0, 1.0, 0.1), Constants.OperatorConstants.TRANSLATION_CURVE, 0.0),
      () -> ExtraMath.processInput(Joystick.deadzone(Constants.OperatorConstants.LEFT_DEADBAND, lx.getAsDouble(), ly.getAsDouble()).getY()  , -ExtraMath.remap(leftTrigger.getAsDouble() , 0.0, 1.0, 1.0, 0.1), Constants.OperatorConstants.TRANSLATION_CURVE, 0.0),
      () -> ExtraMath.processInput(Joystick.deadzone(Constants.OperatorConstants.RIGHT_DEADBAND, rx.getAsDouble(), ry.getAsDouble()).getX(), -ExtraMath.remap(rightTrigger.getAsDouble(), 0.0, 1.0, 1.0, 0.1), Constants.OperatorConstants.ROTATION_CURVE   , 0.0),
      () -> ExtraMath.processInput(Joystick.deadzone(Constants.OperatorConstants.RIGHT_DEADBAND, rx.getAsDouble(), ry.getAsDouble()).getY(), -ExtraMath.remap(rightTrigger.getAsDouble(), 0.0, 1.0, 1.0, 0.1), Constants.OperatorConstants.ROTATION_CURVE   , 0.0),
      () -> this.getDirectAngle(),
      () -> this.getFOD(),
      drive
    );

    drive.setDefaultCommand(driveCommand);
    
  }

  private void configureBindings() {
    resetOdometry.set(false);
    new Trigger(() ->resetOdometry.get()).onTrue(Commands.runOnce(() -> {
      resetOdometry.set(false);
      drive.setPose(Constants.STARTING_POSE);
    }));

    if(RobotBase.isReal()){
      lockPose = new Trigger(driverController::getXButton);
      rstGyro = new Trigger(driverController::getAButton);
      new Trigger(driverController::getLeftStickButton).onTrue(Commands.runOnce(() -> {this.fod = !this.fod;}));
      new Trigger(driverController::getRightStickButton).onTrue(Commands.runOnce(() -> {this.directAngle = !this.directAngle;}));
    } else {
      lockPose = new Trigger(() -> driverController.getRawButton(3));
      rstGyro = new Trigger(() -> driverController.getRawButton(1));
      new Trigger(() -> driverController.getRawButton(9)).onTrue(Commands.runOnce(() -> {this.fod = !this.fod;}));
      new Trigger(() -> driverController.getRawButton(10)).onTrue(Commands.runOnce(() -> {this.directAngle = !this.directAngle;}));

    }
      
      lockPose.whileTrue(Commands.runOnce(() -> {
        drive.stopWithX();
        rumbler.overrideQue(new Rumble(.1, 0.25));
      }, drive).repeatedly());

      rstGyro.onTrue(Commands.runOnce(() -> {
        drive.resetGyro(0);
        rumbler.overrideQue(RumblePreset.TAP.load());
      }));

      new Trigger(() -> TimerHandler.getTeleopRemaining()<Constants.ENDGAME_TIME).onTrue(Commands.runOnce(() -> {
        rumbler.overrideQue(RumblePreset.DOUBLE_TAP.load());;
      }));

      new Trigger(() -> driverController.getPOV() == 180).onTrue(Commands.runOnce(() -> {
        rumbler.overrideQue(RumblePreset.TAP.load());
      }).alongWith(new AnglePresetDriveCommand(
        drive,
        () -> ExtraMath.processInput(Joystick.deadzone(Constants.OperatorConstants.LEFT_DEADBAND, lx.getAsDouble(), ly.getAsDouble()).getX()  , -ExtraMath.remap(leftTrigger.getAsDouble() , 0.0, 1.0, 1.0, 0.1), Constants.OperatorConstants.TRANSLATION_CURVE, 0.0),
        () -> ExtraMath.processInput(Joystick.deadzone(Constants.OperatorConstants.LEFT_DEADBAND, lx.getAsDouble(), ly.getAsDouble()).getY()  , -ExtraMath.remap(leftTrigger.getAsDouble() , 0.0, 1.0, 1.0, 0.1), Constants.OperatorConstants.TRANSLATION_CURVE, 0.0),
        () -> ExtraMath.processInput(Joystick.deadzone(Constants.OperatorConstants.RIGHT_DEADBAND, rx.getAsDouble(), ry.getAsDouble()).getX(), -ExtraMath.remap(rightTrigger.getAsDouble(), 0.0, 1.0, 1.0, 0.1), Constants.OperatorConstants.ROTATION_CURVE   , 0.0),
        () -> ExtraMath.processInput(Joystick.deadzone(Constants.OperatorConstants.RIGHT_DEADBAND, rx.getAsDouble(), ry.getAsDouble()).getY(), -ExtraMath.remap(rightTrigger.getAsDouble(), 0.0, 1.0, 1.0, 0.1), Constants.OperatorConstants.ROTATION_CURVE   , 0.0),
        povDRot
      )));
  }

  public void Periodic(){
    Logger.recordOutput("fieldOrientedDrive", getFOD());
    Logger.recordOutput("directAngle", getDirectAngle());

    rumbler.update(0.02);

    if(drive.getPose().getTranslation().getY() > 4){
      povDRot = DriveConstants.presets.WEST_STATION.getRotation();
    } else {
      povDRot = DriveConstants.presets.EAST_STATION.getRotation();
    }
    //povURot = DriveConstants.presets.PROSSESOR.getRotation();
    //if(drive.getPose().getTranslation().getY() > 4){
    //  povRRot = DriveConstants.presets.WEST_STATION.getRotation();
    //} else {
    //  povRRot = DriveConstants.presets.EAST_STATION.getRotation();
    //}

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

  
  public boolean getFOD(){return fod;}
  public boolean getDirectAngle(){return directAngle;}

}
