/* I wrote this robot code with furry paws on. Just thought I would mention that. -yarden*/

package frc.robot;

import frc.robot.commands.IntakeCommand;
import frc.robot.commands.StationIntake;
import frc.robot.constants.ClimberConstants;
import frc.robot.constants.Constants;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.Constants.OperatorConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.Led;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.BranchSide;
import frc.robot.subsystems.Superstructure.CurrentSuperState;
import frc.robot.subsystems.Superstructure.StationSide;
import frc.robot.subsystems.Superstructure.WantedSuperState;
import frc.robot.subsystems.affector.Affector;
import frc.robot.subsystems.affector.Affector.WantedAffectorState;
import frc.robot.subsystems.affector.elevator.ElevatorIO;
import frc.robot.subsystems.affector.elevator.ElevatorIOSim;
import frc.robot.subsystems.affector.elevator.ElevatorIOSpark;
import frc.robot.subsystems.affector.wrist.WristIO;
import frc.robot.subsystems.affector.wrist.WristIOSim;
import frc.robot.subsystems.affector.wrist.WristIOSpark;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberIO;
import frc.robot.subsystems.climber.ClimberIOSpark;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeIOSpark;
import frc.robot.subsystems.intake.Intake.WantedIntakeState;
import frc.robot.subsystems.physButtons.ButtonIO;
import frc.robot.subsystems.physButtons.ButtonIODIO;
import frc.robot.subsystems.physButtons.ButtonIOSim;
import frc.robot.subsystems.physButtons.Buttons;
import frc.robot.subsystems.swerve.*;
import frc.robot.subsystems.swerve.Drive.WantedDriveState;
import frc.robot.subsystems.swerve.gyro.GyroIO;
import frc.robot.subsystems.swerve.gyro.GyroIOPigeon2;
import frc.robot.subsystems.swerve.gyro.GyroIOSim;
import frc.robot.subsystems.swerve.module.ModuleIO;
import frc.robot.subsystems.swerve.module.ModuleIOSim;
import frc.robot.subsystems.swerve.module.ModuleIOSpark;
import frc.robot.subsystems.vision.CameraIO;
import frc.robot.subsystems.vision.CameraIOPhoton;
import frc.robot.subsystems.vision.CameraIOPhotonSim;
import frc.robot.subsystems.vision.Vision;
import frc.utils.rumble.*;
import frc.utils.VariableLimSLR;
import frc.utils.Joystick.duelJoystickAxis;
import frc.utils.BatteryVoltageSim;
import frc.utils.DisabledInstantCommand;
import frc.utils.ExtraMath;
import frc.utils.Joystick;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import static frc.utils.ControllerMap.*;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggedPowerDistribution;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Alert;
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
  private Affector affector;
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

  private RumbleHandler rumbler = new RumbleHandler(driverController);
  private RumbleHandler opRumbler = new RumbleHandler(operatorController);

  private PowerDistribution pdp = new PowerDistribution(1  , ModuleType.kRev);
  
    
  private VariableLimSLR lxLim = new VariableLimSLR(Double.POSITIVE_INFINITY);
  private VariableLimSLR lyLim = new VariableLimSLR(Double.POSITIVE_INFINITY);
  private VariableLimSLR rxLim = new VariableLimSLR(Double.POSITIVE_INFINITY);
  private VariableLimSLR ryLim = new VariableLimSLR(Double.POSITIVE_INFINITY);

  private double mult = 1.0;
  
  private final Alert driverDisconnected =
  new Alert("Driver controller disconnected (port 0).", AlertType.kWarning);
private final Alert operatorDisconnected =
  new Alert("Operator controller disconnected (port 1).", AlertType.kWarning);

  private Superstructure superstructure;
  
  private duelJoystickAxis driverSticks;

  public RobotContainer() {

    Logger.recordOutput("AScope/zeroPose", new Pose3d());

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
          .withBumperSize(Inches.of(33), Inches.of(35));

      driveSim = new SwerveDriveSimulation(driveTrainSimulationConfig, Constants.STARTING_POSE);
      // Register the drivetrain simulation to the default simulation world
      SimulatedArena.getInstance().addDriveTrainSimulation(driveSim);
    }

    //process driver controls(radial deadzone, curve, trigger slowdown, and inversion)
    driverSticks = new duelJoystickAxis(
        () -> lxLim.calculate(ExtraMath.processInput(Joystick.deadzone(Constants.OperatorConstants.LEFT_DEADBAND,  driverController.getRawAxis(LEFT_STICK_X), driverController.getRawAxis(LEFT_STICK_Y)).getX() ,  -1.0 * ExtraMath.remap(superstructure.wantedState == WantedSuperState.CLIMB ? driverController.getRawAxis(LEFT_TRIGGER)  : 0, 0, 1.0, 1.0, 0.1), Constants.OperatorConstants.TRANSLATION_CURVE, 0.0)),
        () -> lyLim.calculate(ExtraMath.processInput(Joystick.deadzone(Constants.OperatorConstants.LEFT_DEADBAND,  driverController.getRawAxis(LEFT_STICK_X), driverController.getRawAxis(LEFT_STICK_Y)).getY() ,  -1.0 * ExtraMath.remap(superstructure.wantedState == WantedSuperState.CLIMB ? driverController.getRawAxis(LEFT_TRIGGER)  : 0, 0, 1.0, 1.0, 0.1), Constants.OperatorConstants.TRANSLATION_CURVE, 0.0)),
        () -> rxLim.calculate(ExtraMath.processInput(Joystick.deadzone(Constants.OperatorConstants.RIGHT_DEADBAND, driverController.getRawAxis(RIGHT_STICK_X), driverController.getRawAxis(RIGHT_STICK_Y)).getX(), -0.75* ExtraMath.remap(superstructure.wantedState == WantedSuperState.CLIMB ? driverController.getRawAxis(RIGHT_TRIGGER) : 0, 0, 1.0, 1.0, 0.1), Constants.OperatorConstants.ROTATION_CURVE   , 0.0)),
        () -> ryLim.calculate(ExtraMath.processInput(Joystick.deadzone(Constants.OperatorConstants.RIGHT_DEADBAND, driverController.getRawAxis(RIGHT_STICK_X), driverController.getRawAxis(RIGHT_STICK_Y)).getY(), -1.0 * ExtraMath.remap(superstructure.wantedState == WantedSuperState.CLIMB ? driverController.getRawAxis(RIGHT_TRIGGER) : 0, 0, 1.0, 1.0, 0.1), Constants.OperatorConstants.ROTATION_CURVE   , 0.0))
    );


    switch (Constants.MODE) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        vision = new Vision(
          new CameraIOPhoton(VisionConstants.CAMERA_NAMES[0], VisionConstants.BL_ROBOT_TO_CAM),
          new CameraIOPhoton(VisionConstants.CAMERA_NAMES[2], VisionConstants.BR_ROBOT_TO_CAM),
          new CameraIOPhoton(VisionConstants.CAMERA_NAMES[1], VisionConstants.FL_ROBOT_TO_CAM));
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOSpark(0),
                new ModuleIOSpark(1),
                new ModuleIOSpark(2),
                new ModuleIOSpark(3),
                vision,
                driverSticks);
        affector = new Affector(new ElevatorIOSpark(), new WristIOSpark(), operatorController);
        intake = new Intake(new IntakeIOSpark());
        buttons = new Buttons(new ButtonIODIO(4));
        climber = new Climber(new ClimberIOSpark());
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        vision = new Vision(
          new CameraIOPhotonSim(VisionConstants.CAMERA_NAMES[0], VisionConstants.BL_ROBOT_TO_CAM, driveSim::getSimulatedDriveTrainPose),
          new CameraIOPhotonSim(VisionConstants.CAMERA_NAMES[2], VisionConstants.BR_ROBOT_TO_CAM, driveSim::getSimulatedDriveTrainPose),
          new CameraIOPhotonSim(VisionConstants.CAMERA_NAMES[1], VisionConstants.FL_ROBOT_TO_CAM, driveSim::getSimulatedDriveTrainPose)
          );
        if(driveSim != null){
          drive =
              new Drive(
                  new GyroIOSim(driveSim.getGyroSimulation()) {},
                  new ModuleIOSim(driveSim.getModules()[0]),
                  new ModuleIOSim(driveSim.getModules()[1]),
                  new ModuleIOSim(driveSim.getModules()[2]),
                  new ModuleIOSim(driveSim.getModules()[3]),
                  vision,
                  driverSticks
                  );
        affector = new Affector(new ElevatorIOSim(), new WristIOSim(), operatorController);
        affector.setElevHomed(true);
        intake = new Intake(new IntakeIOSim(driveSim, affector));
        buttons = new Buttons(new ButtonIOSim(() -> false));
        climber = new Climber(new ClimberIO() {});
        }
        break;

      default:
        // Replayed robot, disable IO implementations
        vision = new Vision(
          new CameraIO() {},
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
                vision,
                driverSticks
                );
        affector = new Affector(new ElevatorIO() {}, new WristIO() {}, operatorController);
        intake = new Intake(new IntakeIO() {});
        buttons = new Buttons(new ButtonIO() {});
        climber = new Climber(new ClimberIO() {});
        break;
    }

    
    NamedCommands.registerCommand("station", new StationIntake(affector, intake).withTimeout(5));
    NamedCommands.registerCommand("L2", Commands.runOnce(() -> {
      superstructure.setWantedState(WantedSuperState.L2);
      }, affector));
    NamedCommands.registerCommand("L3", Commands.runOnce(() -> {
      superstructure.setWantedState(WantedSuperState.L3);
    }, affector));
  NamedCommands.registerCommand("L4", Commands.runOnce(() -> {
    superstructure.setWantedState(WantedSuperState.L4);
  }, affector));
  NamedCommands.registerCommand("stow", Commands.runOnce(() -> {
    affector.setWantedState(WantedAffectorState.POSITION, Constants.Affector.STOW_POSITION);
  }, affector));

NamedCommands.registerCommand("score", Commands
  .run(() -> intake.setWantedState(WantedIntakeState.INTAKE), intake)
  .until(() -> !intake.isHolding())
  .finallyDo(() -> intake.setWantedState(WantedIntakeState.STOP))
  .withTimeout(2)
);


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
        "Elevator SysId (Quasistatic Forward)", affector.elevSysIdQuasistatic(SysIdRoutine.Direction.kForward).withName("sysid eqf"));
      autoChooser.addOption(
        "Elevator SysId (Quasistatic Reverse)", affector.elevSysIdQuasistatic(SysIdRoutine.Direction.kReverse).withName("sysid eqr"));
      autoChooser.addOption(
        "Elevator SysId (Dynamic Forward)", affector.elevSysIdDynamic(SysIdRoutine.Direction.kForward).withName("sysid edf"));
      autoChooser.addOption(
        "Elevator SysId (Dynamic Reverse)", affector.elevSysIdDynamic(SysIdRoutine.Direction.kReverse).withName("sysid edr"));
      autoChooser.addOption(
        "Wrist SysId (Quasistatic Forward)", affector.wristSysIdQuasistatic(SysIdRoutine.Direction.kForward).withName("sysid wqf"));
      autoChooser.addOption(
        "Wrist SysId (Quasistatic Reverse)", affector.wristSysIdQuasistatic(SysIdRoutine.Direction.kReverse).withName("sysid wqr"));
      autoChooser.addOption(
        "Wrist SysId (Dynamic Forward)", affector.wristSysIdDynamic(SysIdRoutine.Direction.kForward).withName("sysid wdf"));
      autoChooser.addOption(
        "Wrist SysId (Dynamic Reverse)", affector.wristSysIdDynamic(SysIdRoutine.Direction.kReverse).withName("sysid wdr"));
      } 

    superstructure = new Superstructure(
        drive, 
        intake, 
        climber, 
        affector, 
        vision, 
        led, 
        lxLim, 
        lyLim, 
        rxLim, 
        ryLim
    );

    configureBindings();

    LoggedPowerDistribution.getInstance(pdp.getModule(), ModuleType.kRev);
    
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
      // rumbler.overrideQue(new Rumble(.1, 0.25)); // haptic cue not needed
    }, drive).repeatedly());

    //reset angle
    new Trigger(() -> driverController.getRawButton(LOGO_LEFT)).onTrue(new InstantCommand(() -> {
      drive.resetGyro(DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red ? Math.PI : 0);
      rumbler.overrideQue(RumblePreset.TAP.load());
    }));

    //fod toggle
    new Trigger(() -> driverController.getRawButton(LEFT_STICK_BUTTON)).onTrue(new InstantCommand(() -> {
      superstructure.toggleFOD();
    }));

    new Trigger(() -> intake.isHolding() && !intake.wasHolding()).onTrue(new InstantCommand(() -> {
      rumbler.overrideQue(RumblePreset.RING.load());
    //   opRumbler.overrideQue(RumblePreset.TAP.load());
    }));
    //aim to station
    new Trigger(() -> driverController.getRawButton(LB)).onTrue(new InstantCommand(() -> {
      if(superstructure.currentState != CurrentSuperState.CLIMB){
        if(!intake.isHolding()){
          superstructure.alignWithStation(StationSide.LEFT);
        } else {
          superstructure.autoAlign(BranchSide.LEFT);
        }
      }
    }));

    //aim for reef
    new Trigger(() -> driverController.getRawButton(RB)).onTrue(new InstantCommand(() -> {
      if(superstructure.currentState != CurrentSuperState.CLIMB){
        if(!intake.isHolding()){
          superstructure.alignWithStation(StationSide.RIGHT);
        } else {
          superstructure.autoAlign(BranchSide.RIGHT);
        }
      }
    }));
 
    //physical button
    new Trigger(() -> buttons.get(0)).onTrue(new DisabledInstantCommand(() -> {
      affector.resetElevPos(ElevatorConstants.HOME_POS);
      affector.setElevHomed(true);
    }));

    //home
    new Trigger(() -> operatorController.getRawButton(A)).onTrue(new InstantCommand(() -> {
      superstructure.setWantedState(WantedSuperState.HOME);
    }));
        
    //intake controls
    // new Trigger(() -> driverController.getRawButton(RB)).or(() -> operatorController.getRawButton(RB)).whileTrue(new IntakeCommand(intake));
    new Trigger(() -> operatorController.getRawButton(Y)).onTrue(new InstantCommand(() -> {
      intake.setWantedState(WantedIntakeState.OUTTAKE);
    })).onFalse(new InstantCommand(() -> {
      intake.setWantedState(WantedIntakeState.STOP);;
    }));

    new Trigger(() -> driverController.getRawButton(A)).onTrue(new InstantCommand(() -> {
      superstructure.setWantedState(WantedSuperState.L1);
    }));
    new Trigger(() -> driverController.getRawButton(B)).onTrue(new InstantCommand(() -> {
      superstructure.setWantedState(WantedSuperState.L2);
    }));
    new Trigger(() -> driverController.getRawButton(X)).onTrue(new InstantCommand(() -> {
      superstructure.setWantedState(WantedSuperState.L3);
    }));
    new Trigger(() -> driverController.getRawButton(Y)).onTrue(new InstantCommand(() -> {
      superstructure.setWantedState(WantedSuperState.L4);
    }));
    new Trigger(() -> operatorController.getRawButton(LB)).onTrue(new InstantCommand(() -> {
      if(superstructure.currentState != CurrentSuperState.CLIMB){
        superstructure.setWantedState(WantedSuperState.INTAKE_CORAL);
      }
    }));
    new Trigger(() -> driverController.getRawAxis(LEFT_TRIGGER) > 0.5).onTrue(new InstantCommand(() -> {
      if(superstructure.currentState != CurrentSuperState.CLIMB){
        if(!intake.isHolding()){
          superstructure.setWantedState(WantedSuperState.INTAKE_CORAL);
        }
      }
    }));


    new Trigger(() -> operatorController.getPOV() == 0).onTrue(new InstantCommand(() -> {
      affector.setWantedState(WantedAffectorState.POSITION, Constants.Affector.L4_POSITION);
    }));
    new Trigger(() -> operatorController.getPOV() == 90).onTrue(new InstantCommand(() -> {
      affector.setWantedState(WantedAffectorState.POSITION, Constants.Affector.L2_POSITION);
    }));
    new Trigger(() -> operatorController.getPOV() == 180).onTrue(new InstantCommand(() -> {
      affector.setWantedState(WantedAffectorState.POSITION, Constants.Affector.L1_POSITION);
    }));
    new Trigger(() -> operatorController.getPOV() == 270).onTrue(new InstantCommand(() -> {
      affector.setWantedState(WantedAffectorState.POSITION, Constants.Affector.L3_POSITION);
    }));
    new Trigger(() -> operatorController.getPOV() == 270).onTrue(new InstantCommand(() -> {
      affector.setWantedState(WantedAffectorState.POSITION, Constants.Affector.L3_POSITION);
    }));
    new Trigger(() -> operatorController.getRawButton(LOGO_LEFT)).onTrue(new InstantCommand(() -> {
      affector.setWantedState(WantedAffectorState.POSITION, Constants.Affector.STOW_POSITION);
    }));

    // new Trigger(() -> driverController.getRawAxis(LEFT_TRIGGER) > 0.5).whileTrue(new IntakeCommand(intake));
    new Trigger(() -> driverController.getRawAxis(RIGHT_TRIGGER) > 0.5).onTrue(new InstantCommand(() -> {
      if(superstructure.currentState != CurrentSuperState.CLIMB){
        superstructure.score();
      }
    })).onFalse(new InstantCommand(() -> {
      if(superstructure.currentState != CurrentSuperState.CLIMB){
        superstructure.endScore();
      }
    }));
    new Trigger(() -> operatorController.getRawButton(RB)).onTrue(new InstantCommand(() -> {
      superstructure.score();
    }));
    
    new Trigger(() -> driverController.getPOV() == 180).or(() -> operatorController.getRawButton(LOGO_RIGHT)).onTrue(new InstantCommand(() -> {
      superstructure.setWantedState(WantedSuperState.DEFAULT_STATE);
    }));

    new Trigger(() -> driverController.getPOV() == 270).onTrue(new InstantCommand(() -> {
      superstructure.setWantedState(WantedSuperState.CLIMB);
    }));
    new Trigger(() -> driverController.getPOV() == 90).onTrue(new InstantCommand(() -> {
      if(superstructure.wantedState == WantedSuperState.CLIMB){
        superstructure.setWantedState(WantedSuperState.DEFAULT_STATE);
      }
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
    rumbler.update(0.02);
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

    if(Constants.MODE == Constants.RobotMode.SIM){
    //   intake.setHolding(true);
    }

    Command auto = autoChooser.get();
    return auto;
  }

  public void resetDrivetrain(Pose2d pose){
    driveSim.setSimulationWorldPose(pose);
  }
  
  // public boolean getDirectAngle(){return directAngle;}

  public void enable(){
    affector.setElevBrake(true);
    affector.setWristBrake(true);

    affector.setWantedState(WantedAffectorState.POSITION);

    drive.setWantedState(WantedDriveState.TELEOP_DRIVE);
  }



  
}
