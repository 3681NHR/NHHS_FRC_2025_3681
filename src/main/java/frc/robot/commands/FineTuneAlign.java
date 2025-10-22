package frc.robot.commands;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.DriveConstants;
import frc.robot.subsystems.Led;
import frc.robot.subsystems.swerve.Drive;

/**
 * use pathplanner PID to continue driving to target after the main path is
 * finished
 * <p>
 * note: this command does not time out on its own, it is recomended to use a
 * timout decorator
 * 
 * @see <a href=
 *      "https://docs.google.com/document/d/10if4xjAaETTceUVn7l4J-jOCOnm5CJUDS5RAVNIJMQM/edit?tab=t.0">spartronics
 *      whitepaper on auto align</a>
 */
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

        done = drive.getPose().getTranslation()
                .getDistance(target.get().getTranslation()) <= DriveConstants.AUTO_ALIGN_POS_MAX_OFFSET &&
                Math.abs(drive.getPose().getRotation().minus(target.get().getRotation())
                        .getDegrees()) <= DriveConstants.AUTO_ALIGN_ANGLE_MAX_OFFSET;

        led.alignInPos = done;
        Logger.recordOutput("Drive/Align/Fine tune/good", done);
        Logger.recordOutput("Drive/Align/Fine tune/distance to target",
                drive.getPose().getTranslation().getDistance(target.get().getTranslation()));
        Logger.recordOutput("Drive/Align/Fine tune/angle to target",
                Math.abs(drive.getPose().getRotation().minus(target.get().getRotation()).getDegrees()));
        Logger.recordOutput("Drive/Align/Fine tune/good", false);
        Logger.recordOutput("Drive/Align/Fine tune/target", target.get());
    }

    @Override
    public void end(boolean interrupted) {
        led.aligningReef = false;
    }

    @Override
    public boolean isFinished() {
        // interupt if drive is not in drive to point mode
        // otherwise, end when within tolerance
        return done;
    }
}
