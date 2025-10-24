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

    Pair<PathPlannerTrajectory, Command> createExamplePPAuto() {
        
        try{
            PathPlannerPath[] paths = {
                PathPlannerPath.fromPathFile("m4")
            };

        return Pair.of(
                mergeTrajectories(
                    getTraj(paths)
                ),
                Commands.sequence(
                    robotContainer.getDrive().followPath(paths[0])
                ));
        } catch (Exception e){
            throw new RuntimeException("Failed to create Example Auto", e);
        }
    }
    Pair<PathPlannerTrajectory, Command> createExampleChoreoAuto() {
        
        try{
            PathPlannerPath[] paths = {
                PathPlannerPath.fromChoreoTrajectory("2910", 0)
            };

        return Pair.of(
                mergeTrajectories(
                    getTraj(paths)
                ),
                Commands.sequence(
                    robotContainer.getDrive().followPath(paths[0])
                ));
        } catch (Exception e){
            throw new RuntimeException("Failed to create Example Auto", e);
        }
    }
    
    Pair<PathPlannerTrajectory, Command> createR5Auto() {
        try{
            PathPlannerPath[] paths = {
                PathPlannerPath.fromPathFile("r3"),
                PathPlannerPath.fromPathFile("3 rs"),
                PathPlannerPath.fromPathFile("rs 2"),
                PathPlannerPath.fromPathFile("2 rs"),
                PathPlannerPath.fromPathFile("rs 2"),
                PathPlannerPath.fromPathFile("2 rs"),
                PathPlannerPath.fromPathFile("rs 2"),
                PathPlannerPath.fromPathFile("2 rs"),
                PathPlannerPath.fromPathFile("rs 2")
            };

        return Pair.of(
                mergeTrajectories(
                    getTraj(paths)
                ),
                Commands.sequence(
                    robotContainer.getDrive().followPath(paths[0]),
                    robotContainer.getDrive().followPath(paths[1]),
                    robotContainer.getDrive().followPath(paths[2]),
                    robotContainer.getDrive().followPath(paths[3]),
                    robotContainer.getDrive().followPath(paths[4]),
                    robotContainer.getDrive().followPath(paths[5]),
                    robotContainer.getDrive().followPath(paths[6]),
                    robotContainer.getDrive().followPath(paths[7]),
                    robotContainer.getDrive().followPath(paths[8]) 
                ));
        } catch (Exception e){
            throw new RuntimeException("Failed to create R5 Auto", e);
        }
    }
    private PathPlannerTrajectory mergeTrajectories(PathPlannerTrajectory... in){
        List<PathPlannerTrajectoryState> traj = new LinkedList<PathPlannerTrajectoryState>();

        double timeOffset = 0.0;
        for(int i = 0; i < in.length; i++){
            PathPlannerTrajectory trajectory = in[i];
            PathPlannerTrajectoryState[] states = trajectory.getStates().toArray(new PathPlannerTrajectoryState[0]).clone();
            double nextoffset= states[states.length - 1].timeSeconds;
            for(PathPlannerTrajectoryState s1 : states){
                traj.add(s1.copyWithTime(timeOffset + s1.timeSeconds));
            }
            timeOffset += nextoffset;
        }
        return new PathPlannerTrajectory(traj);
    }
    private PathPlannerTrajectory getTraj(PathPlannerPath path){
        return (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red ? path : path).getIdealTrajectory(DriveConstants.PP_CONFIG).get();
    }
    private PathPlannerTrajectory[] getTraj(PathPlannerPath... paths){
        PathPlannerTrajectory[] traj = new PathPlannerTrajectory[paths.length];
        for(int i=0; i<paths.length; i++){
            traj[i] = getTraj(paths[i]);
        }
        return traj;
    }
}