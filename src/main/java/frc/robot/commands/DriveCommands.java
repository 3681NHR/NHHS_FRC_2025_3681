package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.Constants;
import frc.robot.subsystems.swerve.Drive;
import frc.utils.ExtraMath;
import frc.utils.Joystick;

import static frc.robot.constants.DriveConstants.*;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

public class DriveCommands {

  private DriveCommands() {}

  private static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
    // Apply deadband
    double linearMagnitude = Math.hypot(x, y);
    Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

    // Return new linear velocity
    return new Pose2d(new Translation2d(), linearDirection)
        .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
        .getTranslation();
  }

  /**
   * Field relative drive command using two joysticks (controlling linear and angular velocities).
   */
  public static Command joystickDrive(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {
    return Commands.run(
        () -> {
            joystickDriveFunc(drive, xSupplier, ySupplier, omegaSupplier);
        },
        drive);
  }

  public static void joystickDriveFunc(
    Drive drive,
    DoubleSupplier xSupplier,
    DoubleSupplier ySupplier,
    DoubleSupplier omegaSupplier) {
    // Get linear velocity
    Translation2d linearVelocity =
        getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

    double omega = omegaSupplier.getAsDouble();

    // Convert to field relative speeds & send command
    ChassisSpeeds speeds =
        new ChassisSpeeds(
            linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
            linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
            omega * drive.getMaxAngularSpeedRadPerSec());
    boolean isFlipped =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == Alliance.Red;
    drive.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            speeds,
            isFlipped
                ? drive.getRotation().plus(new Rotation2d(Math.PI))
                : drive.getRotation()));

}
  
  /**
   * Field relative drive command using joystick for linear control and PID for angular control.
   * Possible use cases include snapping to an angle, aiming at a vision target, or controlling
   * absolute rotation with a joystick.
   */
  public static Command joystickDriveAtAngle(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      Supplier<Rotation2d> rotationSupplier) {

    // Construct command
    return Commands.run(
            () -> {
                joystickDriveAtAngleFunc(drive, xSupplier, ySupplier, rotationSupplier);
            },
            drive);
  }

  public static void joystickDriveAtAngleFunc(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      Supplier<Rotation2d> rotationSupplier) {

    // Create PID controller
    ProfiledPIDController angleController =
        new ProfiledPIDController(
            ANGLE_P,
            0.0,
            ANGLE_D,
            new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));
    
    angleController.reset(drive.getRotation().getRadians());

    angleController.enableContinuousInput(0, 2*Math.PI);
    
    // Calculate angular speed
    double omega =
        angleController.calculate(
            drive.getRotation().getRadians(), rotationSupplier.get().getRadians());
    
    joystickDriveFunc(drive, xSupplier, ySupplier, () -> omega);
  }
  
  public static Command driveCommand(DoubleSupplier tx, DoubleSupplier ty, DoubleSupplier rx, DoubleSupplier ry, BooleanSupplier direct, BooleanSupplier fod, Drive drive){
    return Commands.run(() -> {
        if(fod.getAsBoolean()){
            if(direct.getAsBoolean()){
                joystickDriveAtAngleFunc(drive, ty, tx, () -> ExtraMath.getAngle(//use angle deadzone if in D/A
                    Joystick.deadzone(Constants.OperatorConstants.ANGLE_DEADBAND,  ry.getAsDouble(), rx.getAsDouble()).getX(), 
                    Joystick.deadzone(Constants.OperatorConstants.ANGLE_DEADBAND,  ry.getAsDouble(), rx.getAsDouble()).getY()
                ));
            } else {
                joystickDriveFunc(drive, ty, tx, rx);
            }
        } else {
                drive.runVelocity(new ChassisSpeeds(
                    ty.getAsDouble()*drive.getMaxLinearSpeedMetersPerSec(), 
                    tx.getAsDouble()*drive.getMaxLinearSpeedMetersPerSec(), 
                    rx.getAsDouble()*drive.getMaxAngularSpeedRadPerSec()
                ));
        }
    }, drive);

  }
}