package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.arm.ArmSubsystem.State;

public class OpperatorDefaultControlCommand extends Command {

  ArmSubsystem arm;
  XboxController x;

  public OpperatorDefaultControlCommand(ArmSubsystem a, XboxController x) {
    arm = a;
    this.x = x;
    addRequirements(arm);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if(x.getLeftTriggerAxis()>=Constants.OperatorConstants.OPERATOR_TRIGGER_DEADBAND || x.getRightTriggerAxis()>=Constants.OperatorConstants.OPERATOR_TRIGGER_DEADBAND){
      arm.setState(State.MAN);
      if(x.getLeftBumperButton()){
        arm.setExtention(arm.getExtentionSetpoint() + (x.getRightTriggerAxis()-x.getLeftTriggerAxis())*Constants.OperatorConstants.ARM_EXTENTION_SENSITIVITY);
      } else if(x.getRightBumperButton()){
        arm.setWrist(arm.getWristSetpoint() + (x.getRightTriggerAxis()-x.getLeftTriggerAxis())*Constants.OperatorConstants.WRIST_ANGLE_SENSITIVITY);
      } else {
        arm.setAngle(arm.getAngleSetpoint() + (x.getRightTriggerAxis()-x.getLeftTriggerAxis())*Constants.OperatorConstants.ARM_ANGLE_SENSITIVITY);
      }
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
