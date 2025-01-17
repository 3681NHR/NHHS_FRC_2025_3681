package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.OpperatorDefaultControlCommand;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.arm.ArmIONeo;
import frc.robot.subsystems.arm.ArmIOSim;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.utils.rumble.ControllerRumble;
import frc.utils.rumble.RumbleType;
import frc.utils.TimerHandler;
import frc.utils.ExtraMath;
import frc.utils.rumble.RumblePreset;

import java.io.File;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralAlgaeStack;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {


  private final SwerveDriveSubsystem swerveDriveSubsystem = new SwerveDriveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
  private final ArmSubsystem armSubsystem;

  private final OpperatorDefaultControlCommand opperatorDefaultControlCommand;
  
  private final LoggedDashboardChooser<Command> autoChooser;

  private boolean fod = Constants.drive.STARTING_FOD;
  private boolean directAngle = Constants.drive.STARTING_DIRECT_ANGLE;

  private Trigger lockPose;
  private Trigger rstGyro;

  private Alert brownout = new Alert("brownout", AlertType.kError);

  private final XboxController m_driverController =
      new XboxController(OperatorConstants.DRIVER_CONTROLLER_PORT);

  private final XboxController m_operatorController =
      new XboxController(OperatorConstants.OPERATOR_CONTROLLER_PORT);

  private ControllerRumble rumbler = new ControllerRumble(m_driverController);

  private PowerDistribution pdp = new PowerDistribution();

  public RobotContainer() {
    if(RobotBase.isReal()){
      armSubsystem = new ArmSubsystem(new ArmIONeo());
    } else {
      armSubsystem = new ArmSubsystem(new ArmIOSim());
    }


    configureBindings();

    SmartDashboard.putData("PDP", pdp);

    
    // left stick controls translation
    // right stick controls the angular velocity of the robot
    // sim command used raw axis for simulating joysticks
    // bumpers and triggers control center of rotation, usefull for evasive meneuvers
    Command driveAngulerVelocity = swerveDriveSubsystem.driveCommand(
      () -> ExtraMath.processInput(m_driverController.getLeftY() , -ExtraMath.remap(m_driverController.getLeftTriggerAxis() , 0.0, 1.0, 1.0, 0.1), Constants.OperatorConstants.TRANSLATION_CURVE, OperatorConstants.LEFT_Y_DEADBAND),
      () -> ExtraMath.processInput(m_driverController.getLeftX() , -ExtraMath.remap(m_driverController.getLeftTriggerAxis() , 0.0, 1.0, 1.0, 0.1), Constants.OperatorConstants.TRANSLATION_CURVE, OperatorConstants.LEFT_X_DEADBAND),
      () -> ExtraMath.processInput(m_driverController.getRightX(), -ExtraMath.remap(m_driverController.getRightTriggerAxis(), 0.0, 1.0, 1.0, 0.1), Constants.OperatorConstants.ROTATION_CURVE, OperatorConstants.RIGHT_X_DEADBAND),
      () -> ExtraMath.processInput(m_driverController.getRightY(), -ExtraMath.remap(m_driverController.getRightTriggerAxis(), 0.0, 1.0, 1.0, 0.1), Constants.OperatorConstants.ROTATION_CURVE, OperatorConstants.RIGHT_X_DEADBAND),
      () -> this.getDirectAngle(),
      () -> this.getFOD()
    );
    // left stick controls translation
    // right stick controls the angular velocity of the robot
    // sim command used raw axis for simulating joysticks
    // bumpers and triggers control center of rotation, usefull for evasive meneuvers
    Command driveAngulerVelocitySim = swerveDriveSubsystem.driveCommand(
      () -> ExtraMath.processInput(m_driverController.getRawAxis(1), -ExtraMath.remap(m_driverController.getRawAxis(2), 0.0, 1.0, 1.0, 0.1), Constants.OperatorConstants.TRANSLATION_CURVE, OperatorConstants.LEFT_Y_DEADBAND),
      () -> ExtraMath.processInput(m_driverController.getRawAxis(0), -ExtraMath.remap(m_driverController.getRawAxis(2), 0.0, 1.0, 1.0, 0.1), Constants.OperatorConstants.TRANSLATION_CURVE, OperatorConstants.LEFT_X_DEADBAND),
      () -> ExtraMath.processInput(m_driverController.getRawAxis(4), -ExtraMath.remap(m_driverController.getRawAxis(3), 0.0, 1.0, 1.0, 0.1), Constants.OperatorConstants.ROTATION_CURVE, OperatorConstants.RIGHT_X_DEADBAND),
      () -> ExtraMath.processInput(m_driverController.getRawAxis(5), -ExtraMath.remap(m_driverController.getRawAxis(3), 0.0, 1.0, 1.0, 0.1), Constants.OperatorConstants.ROTATION_CURVE, OperatorConstants.RIGHT_X_DEADBAND),
      () -> this.getDirectAngle(),
      () -> this.getFOD()
    );

    swerveDriveSubsystem.setDefaultCommand(RobotBase.isReal() ? driveAngulerVelocity : driveAngulerVelocitySim);

    opperatorDefaultControlCommand = new OpperatorDefaultControlCommand(armSubsystem, m_operatorController);

    armSubsystem.setDefaultCommand(opperatorDefaultControlCommand);

    autoChooser = new LoggedDashboardChooser<Command>("auto", AutoBuilder.buildAutoChooser());

    SmartDashboard.putData("arm", armSubsystem);

    SimulatedArena.getInstance().addGamePiece(new ReefscapeCoralAlgaeStack(new Translation2d(1.220, 5.85)));
    SimulatedArena.getInstance().addGamePiece(new ReefscapeCoralAlgaeStack(new Translation2d(1.220, 4)));
    SimulatedArena.getInstance().addGamePiece(new ReefscapeCoralAlgaeStack(new Translation2d(1.220, 2)));

    SimulatedArena.getInstance().addGamePiece(new ReefscapeCoralAlgaeStack(new Translation2d(15.690, 5.85)));
    SimulatedArena.getInstance().addGamePiece(new ReefscapeCoralAlgaeStack(new Translation2d(15.690, 4)));
    SimulatedArena.getInstance().addGamePiece(new ReefscapeCoralAlgaeStack(new Translation2d(15.690, 2)));
    
  }

  private void configureBindings() {
    //run lock command constantly instead of driver input
    SmartDashboard.putBoolean("reset odometry", false);
    new Trigger(() ->SmartDashboard.getBoolean("reset odometry", false)).onTrue(Commands.runOnce(() -> {
      SmartDashboard.putBoolean("reset odometry", false);
      swerveDriveSubsystem.resetOdometry(new Pose2d());
    }));
    if(RobotBase.isReal()){
      lockPose = new Trigger(m_driverController::getXButton);
      rstGyro = new Trigger(m_driverController::getAButton);
      new Trigger(m_driverController::getLeftStickButton).onTrue(Commands.runOnce(() -> {this.fod = !this.fod;}));
      new Trigger(m_driverController::getRightStickButton).onTrue(Commands.runOnce(() -> {this.directAngle = !this.directAngle;}));
      new Trigger(m_driverController::getYButton).onTrue(Commands.runOnce(() -> rumbler.addRumble(RumblePreset.RING.load(), RumbleType.OVERLAY)));
    
    
    } else {
      lockPose = new Trigger(() -> m_driverController.getRawButton(3));
      rstGyro = new Trigger(() -> m_driverController.getRawButton(1));
      new Trigger(() -> m_driverController.getRawButton(9)).onTrue(Commands.runOnce(() -> {this.fod = !this.fod;}));
      new Trigger(() -> m_driverController.getRawButton(10)).onTrue(Commands.runOnce(() -> {this.directAngle = !this.directAngle;}));
    }
      lockPose.whileTrue(Commands.runOnce(swerveDriveSubsystem::lock, swerveDriveSubsystem).alongWith(Commands.runOnce(() -> rumbler.addRumble(0.1, 0.1, RumbleType.OVERLAY))).repeatedly());
      rstGyro.onTrue(Commands.runOnce(swerveDriveSubsystem::zeroGyro, swerveDriveSubsystem).alongWith(Commands.runOnce(() -> rumbler.addRumble(RumblePreset.TAP.load(), RumbleType.OVERLAY))));

      new Trigger(() -> TimerHandler.getTeleopRemaining()<30.0).or(
        new Trigger(() -> TimerHandler.getAutoRemaining()<3.0)
      ).onTrue(Commands.runOnce(() -> {
        rumbler.addRumble(RumblePreset.DOUBLE_TAP.load(), RumbleType.OVERRIDE);
      }));

      new Trigger(() -> m_operatorController.getPOV()==180).onTrue(Commands.runOnce(() -> armSubsystem.setPosition(Constants.arm.positions.CORAL_GROUND)));
      new Trigger(() -> m_operatorController.getPOV()==0).onTrue(Commands.runOnce(() -> armSubsystem.setPosition(Constants.arm.positions.CORAL_STATION)));
      new Trigger(() -> m_operatorController.getPOV()==90).onTrue(Commands.runOnce(() -> armSubsystem.setPosition(Constants.arm.positions.CORAL_L1)));
      new Trigger(() -> m_operatorController.getPOV()==270).onTrue(Commands.runOnce(() -> armSubsystem.setPosition(Constants.arm.positions.CORAL_L3)));
    }

  public void Periodic(){
    SmartDashboard.putBoolean("fod", getFOD());
    SmartDashboard.putBoolean("direct angle", directAngle);
    SmartDashboard.putNumber("voltage", pdp.getVoltage());

    if(pdp.getVoltage() <= 8){
      brownout.set(true);
    } else {
      brownout.set(false);
    }

    rumbler.update();

  }
  public void SimPeriodic(){
  }

  public Command getAutonomousCommand() {
    Command auto = autoChooser.get();
    return auto;
  }

  
  public boolean getFOD(){return fod;}
  public boolean getDirectAngle(){return directAngle;}

}
