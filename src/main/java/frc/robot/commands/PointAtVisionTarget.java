package frc.robot.commands;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.swerve.Drive;
import frc.robot.subsystems.vision.Vision;
import frc.utils.ExtraMath;
import frc.utils.Joystick.duelJoystickAxis;

public class PointAtVisionTarget extends Command {

  Drive drive;
  Vision  vision;

  duelJoystickAxis sticks;

  int tagID = -1;

  PIDController pid = new PIDController(VisionConstants.ANGLE_P, 0, VisionConstants.ANGLE_D);

  Optional<Double> yaw;

  /**
   * drive at given angle
   * @param drive
   * @param tx
   * @param ty
   * @param rx
   * @param ry
   * @param angle
   */
  public PointAtVisionTarget(duelJoystickAxis sticks, Drive drive, Vision vision, int tagID) {
    this.drive = drive;
    this.vision = vision;
    this.tagID = tagID;
    this.sticks = sticks;
    addRequirements(drive);
    addRequirements(vision);
  }


  @Override
  public void initialize() {
    
  }


  @Override
  public void execute() {
    yaw = vision.getYaw(tagID);

    if(yaw.isPresent()){
      Logger.recordOutput("Drive/yawToVisionTarget", yaw.get());
    }

    DriveCommands.joystickDriveFunc(drive, sticks.lx, sticks.ly, () -> MathUtil.clamp(pid.calculate(yaw.isPresent() ? yaw.get() : 0, 0), -1, 1));
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return ExtraMath.getMagnitude(sticks.rx.getAsDouble(), sticks.ry.getAsDouble()) < Constants.OperatorConstants.ANGLE_DEADBAND;
  }
}
