package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.swerve.Drive;
import frc.utils.ExtraMath;

public class AnglePresetDriveCommand extends Command {

  Drive drive;
  Rotation2d angle;

  DoubleSupplier tx;
  DoubleSupplier ty;
  DoubleSupplier rx;
  DoubleSupplier ry;

  /**
   * drive at given angle
   * @param drive
   * @param tx
   * @param ty
   * @param rx
   * @param ry
   * @param angle
   */
  public AnglePresetDriveCommand(Drive drive, DoubleSupplier tx, DoubleSupplier ty, DoubleSupplier rx, DoubleSupplier ry, Rotation2d angle) {
    this.drive = drive;
    this.angle = angle;
    this.tx = tx;
    this.ty = ty;
    this.rx = rx;
    this.ry = ry;
    addRequirements(drive);
    
  }


  @Override
  public void initialize() {

  }


  @Override
  public void execute() {
    DriveCommands.joystickDriveAtAngleFunc(drive, tx, ty, () -> angle);
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return ExtraMath.getMagnitude(rx.getAsDouble(), ry.getAsDouble()) < Constants.OperatorConstants.ANGLE_DEADBAND;
  }
}
