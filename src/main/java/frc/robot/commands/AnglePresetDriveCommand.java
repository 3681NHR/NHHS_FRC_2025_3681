package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.swerve.Drive;
import frc.utils.ExtraMath;
import frc.utils.Joystick.duelJoystickAxis;

public class AnglePresetDriveCommand extends Command {

  Drive drive;
  Supplier<Rotation2d> angle;

  duelJoystickAxis sticks;

  /**
   * drive at given angle
   * @param drive
   * @param tx
   * @param ty
   * @param rx
   * @param ry
   * @param angle
   */
  public AnglePresetDriveCommand(duelJoystickAxis sticks, Drive drive, Supplier<Rotation2d> angle) {
    this.drive = drive;
    this.angle = angle;
    this.sticks = sticks;
    addRequirements(drive);
    
  }


  @Override
  public void initialize() {

  }


  @Override
  public void execute() {
    DriveCommands.joystickDriveAtAngleFunc(drive, sticks.lx, sticks.ly, angle);
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return ExtraMath.getMagnitude(sticks.rx.getAsDouble(), sticks.ry.getAsDouble()) < Constants.OperatorConstants.ANGLE_DEADBAND;
  }
}
