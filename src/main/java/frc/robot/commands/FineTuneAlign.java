package frc.robot.commands;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.DriveConstants;
import frc.robot.subsystems.Led;
import frc.robot.subsystems.swerve.Drive;
import frc.robot.subsystems.swerve.Drive.CurrentDriveState;

public class FineTuneAlign extends Command {

    private Supplier<Pose2d> target;
    private Drive drive;
    private Led led;

    private PathPlannerTrajectoryState state;

    private boolean done = false;

    public FineTuneAlign(Supplier<Pose2d> target, Drive drive, Led led) {
        this.target = target;
        this.drive = drive;
        this.led = led;

        
        addRequirements(drive);
    }

    @Override
    public void initialize() {

        state = new PathPlannerTrajectoryState();
        state.pose = target.get();
    }

    @Override
    public void execute() {

        drive.runVelocity(drive.autoController.calculateRobotRelativeSpeeds(drive.getPose(), state));


        done = drive.getPose().getTranslation().getDistance(target.get().getTranslation()) <= DriveConstants.AUTO_ALIGN_POS_MAX_OFFSET &&
            Math.abs(drive.getPose().getRotation().minus(target.get().getRotation()).getDegrees()) <= DriveConstants.AUTO_ALIGN_ANGLE_MAX_OFFSET;
        
        led.alignInPos = done;
        Logger.recordOutput("Drive/Align/Fine tune/good"   , done);
        Logger.recordOutput("Drive/Align/Fine tune/distance to target", drive.getPose().getTranslation().getDistance(target.get().getTranslation()));
        Logger.recordOutput("Drive/Align/Fine tune/angle to target"   , Math.abs(drive.getPose().getRotation().minus(target.get().getRotation()).getDegrees()));
        Logger.recordOutput("Drive/Align/Fine tune/good"   , false);
        Logger.recordOutput("Drive/Align/Fine tune/target"   , target.get());
    }

    @Override
    public void end(boolean interrupted) {
        led.aligningReef = false;
    }

    @Override
    public boolean isFinished() {
        return done || drive.currentState != CurrentDriveState.DRIVE_TO_POINT;
    }
}
