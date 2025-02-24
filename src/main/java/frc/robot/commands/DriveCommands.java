package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.Constants;
import frc.robot.subsystems.swerve.Drive;
import frc.utils.ExtraMath;
import frc.utils.Joystick;
import frc.utils.Joystick.duelJoystickAxis;

import static frc.robot.constants.DriveConstants.*;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

public class DriveCommands {

    
    // Create PID controller
    static PIDController angleController =
        new PIDController(
        RobotBase.isReal() ? ANGLE_P : ANGLE_SIM_P, 
            0.0,
            RobotBase.isReal() ? ANGLE_D : ANGLE_SIM_D);

    static double rx = 0;
    static double ry = 0;

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

    ChassisSpeeds desiredChassisSpeeds = 
    ChassisSpeeds.fromFieldRelativeSpeeds(
        speeds,
        isFlipped
            ? drive.getRotation().plus(new Rotation2d(Math.PI))
            : drive.getRotation());
     //       Compensate for gyro drift by adjusting the target chassis speeds
    var angularVelocity = new Rotation2d(drive.getAngulerVelocity() * ANGULAR_VELOCITY_COEFFICIENT);
    if (angularVelocity.getRadians() != 0.0) {
        desiredChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                desiredChassisSpeeds,
                angularVelocity);
    }

    drive.runVelocity(desiredChassisSpeeds);

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
            drive).beforeStarting(() -> {
                angleController.reset();
                angleController.enableContinuousInput(-Math.PI, Math.PI);
            }, drive);
  }

  public static void joystickDriveAtAngleFunc(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      Supplier<Rotation2d> rotationSupplier) {

    // Calculate angular speed
    double omega =
    MathUtil.clamp(
        angleController.calculate(
            drive.getRotation().getRadians(), rotationSupplier.get().rotateBy(DriverStation.getAlliance().isPresent()&& DriverStation.getAlliance().get() == Alliance.Red ? Rotation2d.k180deg : new Rotation2d()).getRadians()),
    -ANGLE_MAX_VELOCITY, ANGLE_MAX_VELOCITY);

    Logger.recordOutput("angletarget", angleController.getSetpoint());
    
    joystickDriveFunc(drive, xSupplier, ySupplier, () -> omega);
  }
  
  public static Command driveCommand(duelJoystickAxis sticks, BooleanSupplier direct, BooleanSupplier fod, Drive drive){
    return Commands.run(() -> {
        if(fod.getAsBoolean()){
            if(direct.getAsBoolean()){
                if(Joystick.deadzone(Constants.OperatorConstants.ANGLE_DEADBAND,  sticks.ry.getAsDouble(), sticks.rx.getAsDouble()).getX() != 0){
                    rx = sticks.rx.getAsDouble();
                    ry = sticks.ry.getAsDouble();
                }

                joystickDriveAtAngleFunc(drive, sticks.ly, sticks.lx, () -> ExtraMath.getAngle(//use angle deadzone if in D/A
                    Joystick.deadzone(Constants.OperatorConstants.ANGLE_DEADBAND,  ry, rx).getX(), 
                    Joystick.deadzone(Constants.OperatorConstants.ANGLE_DEADBAND,  ry, rx).getY()
                ));
            } else {
                joystickDriveFunc(drive, sticks.ly, sticks.lx, sticks.rx);
            }
        } else {
                drive.runVelocity(new ChassisSpeeds(
                    sticks.ly.getAsDouble()*drive.getMaxLinearSpeedMetersPerSec(), 
                    sticks.lx.getAsDouble()*drive.getMaxLinearSpeedMetersPerSec(), 
                    sticks.rx.getAsDouble()*drive.getMaxAngularSpeedRadPerSec()
                ));
        }
    }, drive).beforeStarting(() -> {
        rx = sticks.rx.getAsDouble();
        ry = sticks.ry.getAsDouble();

        angleController.reset();
        angleController.enableContinuousInput(-Math.PI, Math.PI);
    }, drive);

  }

  public static void resetPid(Drive drive){
    angleController.reset();
  }
}