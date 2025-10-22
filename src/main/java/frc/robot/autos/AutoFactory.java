package frc.robot.autos;

import java.util.LinkedList;
import java.util.List;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.RobotContainer;
import frc.robot.constants.DriveConstants;

/**
 * A factory for creating autonomous programs for a given {@link Auto}
 */
public class AutoFactory {

    private final RobotContainer robotContainer;
    /**
     * Create a new <code>AutoFactory</code>.
     *
     * @param robotContainer The {@link RobotContainer}
     */
    public AutoFactory(final RobotContainer robotContainer) {
        this.robotContainer = robotContainer;
    }

    /* Autonomous program factories
     *
     * Factory methods should be added here for each autonomous program.
     * The factory methods must:
     *   1. Be package-private (i.e. no access modifier)
     *   2. Accept no parameters
     *   3. Return a link Command
     */
    private static final Command IDLE_COMMAND = Commands.idle();

    Pair<PathPlannerTrajectory, Command> createIdleAuto() {
        return Pair.of(null, IDLE_COMMAND);
    }

    Pair<PathPlannerTrajectory, Command> createExampleAuto() {
        return Pair.of(
                null,
                Commands.sequence());
    }
    
    Pair<PathPlannerTrajectory, Command> createTestAuto() {
        try{
            PathPlannerPath path = PathPlannerPath.fromPathFile("m4");

            List<PathPlannerTrajectoryState> traj = new LinkedList<PathPlannerTrajectoryState>();

            traj.addAll((DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red ? path.flipPath() : path).getIdealTrajectory(DriveConstants.PP_CONFIG).get().getStates());

            for(int i=0; i<20-traj.size(); i++){
                traj.add(traj.get(traj.size()-1));
            }
        return Pair.of(
                new PathPlannerTrajectory(traj),
                Commands.sequence(
                    robotContainer.getDrive().followPath(path)
                ));
        } catch (Exception e){
            throw new RuntimeException("Failed to create Test Auto", e);
        }
    }
    
    Pair<PathPlannerTrajectory, Command> createR5Auto() {
        try{
            PathPlannerPath[] paths = {
                PathPlannerPath.fromPathFile("r3"),
                PathPlannerPath.fromPathFile("3 rs"),
                PathPlannerPath.fromPathFile("rs 2"),
                PathPlannerPath.fromPathFile("2 rs"),
            };

        return Pair.of(
                mergeTrajectories(
                    getTraj(paths[0]),
                    getTraj(paths[1]),
                    getTraj(paths[2]),
                    getTraj(paths[3])
                ),
                Commands.sequence(
                    robotContainer.getDrive().followPath(paths[0]),
                    robotContainer.getDrive().followPath(paths[1]),
                    robotContainer.getDrive().followPath(paths[2]),
                    robotContainer.getDrive().followPath(paths[3])
                ));
        } catch (Exception e){
            throw new RuntimeException("Failed to create Test Auto", e);
        }
    }
    
    Pair<PathPlannerTrajectory, Command> createEAuto() {
        try{
            PathPlannerPath[] paths = {
                PathPlannerPath.fromPathFile("New Path")
            };

            List<PathPlannerTrajectoryState> traj = new LinkedList<PathPlannerTrajectoryState>();

            double timeOffset = 0.0;
            for(int i = 0; i < paths.length; i++){
                PathPlannerPath path = paths[i];
                PathPlannerTrajectoryState[] states = (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red ? path.flipPath() : path).getIdealTrajectory(DriveConstants.PP_CONFIG).get().getStates().toArray(new PathPlannerTrajectoryState[0]);
                double nextoffset= states[states.length - 1].timeSeconds;
                for(PathPlannerTrajectoryState s : states){
                    s.timeSeconds += timeOffset;
                    traj.add(s);
                }
                timeOffset += nextoffset;
            }
        return Pair.of(
                new PathPlannerTrajectory(traj),
                Commands.sequence(
                    robotContainer.getDrive().followPath(paths[0])
                ));
        } catch (Exception e){
            throw new RuntimeException("Failed to create Test Auto", e);
        }
    }

    private PathPlannerTrajectory mergeTrajectories(PathPlannerTrajectory... in){
        List<PathPlannerTrajectoryState> traj = new LinkedList<PathPlannerTrajectoryState>();

        double timeOffset = 0.0;
        for(int i = 0; i < in.length; i++){
            PathPlannerTrajectory trajectory = in[i];
            PathPlannerTrajectoryState[] states = trajectory.getStates().toArray(new PathPlannerTrajectoryState[0]);
            double nextoffset= states[states.length - 1].timeSeconds;
            for(PathPlannerTrajectoryState s : states){
                s.timeSeconds += timeOffset;
                traj.add(s);
            }
            timeOffset += nextoffset;
        }
        return new PathPlannerTrajectory(traj);
    }
    private PathPlannerTrajectory getTraj(PathPlannerPath path){
        return (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red ? path.flipPath() : path).getIdealTrajectory(DriveConstants.PP_CONFIG).get();
    }
}