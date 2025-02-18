package frc.robot.commands;

import java.util.Optional;
import java.util.function.IntSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.swerve.Drive;
import frc.robot.subsystems.vision.Vision;
import frc.utils.ExtraMath;
import frc.utils.Joystick.duelJoystickAxis;

public class PointAtVisionTarget extends Command {

  Drive drive;
  Vision  vision;

  duelJoystickAxis sticks;

  IntSupplier tagID = () ->-1;

  ProfiledPIDController pid = new ProfiledPIDController(
    RobotBase.isReal() ? VisionConstants.ANGLE_P : VisionConstants.ANGLE_SIM_P, 
    0, 
    RobotBase.isReal() ? VisionConstants.ANGLE_D : VisionConstants.ANGLE_SIM_D,
    new TrapezoidProfile.Constraints(Units.radiansToDegrees(DriveConstants.ANGLE_MAX_VELOCITY), Units.radiansToDegrees(DriveConstants.ANGLE_MAX_ACCELERATION))
  );

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
  public PointAtVisionTarget(duelJoystickAxis sticks, Drive drive, Vision vision, IntSupplier tagID) {
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
    yaw = vision.getYaw(tagID.getAsInt());

    yaw = Optional.of(yaw.isPresent() ? yaw.get() : 5);

    if(yaw.isPresent()){
      Logger.recordOutput("Drive/yawToVisionTarget", yaw.get());
    }

    DriveCommands.joystickDriveFunc(drive, sticks.ly, sticks.lx, () -> pid.calculate(yaw.get(), 0));
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return ExtraMath.getMagnitude(sticks.rx.getAsDouble(), sticks.ry.getAsDouble()) > Constants.OperatorConstants.ANGLE_DEADBAND;
  }
}
