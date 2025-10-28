package frc.robot.autos;

import java.util.LinkedList;
import java.util.List;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.RobotContainer;
import frc.robot.constants.DriveConstants;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.BranchSide;
import frc.robot.subsystems.Superstructure.WantedSuperState;
import frc.robot.subsystems.affector.Affector.AffectorPosition;
import frc.robot.subsystems.affector.Affector.WantedAffectorState;

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

    /*
     * Autonomous program factories
     *
     * Factory methods should be added here for each autonomous program.
     * The factory methods must:
     * 1. Be package-private (i.e. no access modifier)
     * 2. Accept no parameters
     * 3. Return a link Command
     */
    private static final Command IDLE_COMMAND = Commands.idle();

    Pair<PathPlannerTrajectory, Command> createIdleAuto() {
        return Pair.of(null, IDLE_COMMAND);
    }

    public enum ReefLevel {
        L1,
        L2,
        L3,
        L4
    }

    Pair<PathPlannerTrajectory, Command> createExamplePPAuto() {

        try {
            PathPlannerPath[] paths = {
                    PathPlannerPath.fromPathFile("m4")
            };

            return Pair.of(
                    mergeTrajectories(
                            getTraj(paths)),
                    Commands.sequence(
                            robotContainer.getDrive().followPath(paths[0])));
        } catch (Exception e) {
            throw new RuntimeException("Failed to create Example Auto", e);
        }
    }

    Pair<PathPlannerTrajectory, Command> createExampleChoreoAuto() {

        try {
            PathPlannerPath[] paths = {
                    PathPlannerPath.fromChoreoTrajectory("2910", 0)
            };

            return Pair.of(
                    mergeTrajectories(
                            getTraj(paths)),
                    Commands.sequence(
                            robotContainer.getDrive().followPath(paths[0]),
                            new InstantCommand(() -> {
                                robotContainer.getAffector().setWantedState(WantedAffectorState.POSITION,
                                        new AffectorPosition(1.0, 0));
                                robotContainer.getLed().override = true;
                            }))
                            .finallyDo(() -> {
                                robotContainer.getLed().override = false;
                            }));
        } catch (Exception e) {
            throw new RuntimeException("Failed to create Example Auto", e);
        }
    }

    Pair<PathPlannerTrajectory, Command> createR5Auto() {
        try {
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
                            getTraj(paths)),
                    Commands.sequence(
                            robotContainer.getDrive().followPath(paths[0]),
                            alignThenScore(BranchSide.LEFT, ReefLevel.L4),
                            station(robotContainer.getDrive().followPath(paths[1]), 1.0, 3.0),
                            robotContainer.getDrive().followPath(paths[2]),
                            alignThenScore(BranchSide.LEFT, ReefLevel.L4),
                            station(robotContainer.getDrive().followPath(paths[3]), 1.0, 3.0),
                            robotContainer.getDrive().followPath(paths[4]),
                            alignThenScore(BranchSide.RIGHT, ReefLevel.L4),
                            station(robotContainer.getDrive().followPath(paths[5]), 1.0, 3.0),
                            robotContainer.getDrive().followPath(paths[6]),
                            alignThenScore(BranchSide.LEFT, ReefLevel.L3),
                            station(robotContainer.getDrive().followPath(paths[3]), 1.0, 3.0),
                            robotContainer.getDrive().followPath(paths[8]),
                            alignThenScore(BranchSide.RIGHT, ReefLevel.L3)
                            ));
        } catch (Exception e) {
            throw new RuntimeException("Failed to create R5 Auto", e);
        }
    }
    public Pair<PathPlannerTrajectory, Command> test(){
        return Pair.of(null, robotContainer.getSuperstructure().getAutoAlignLeft());//FIXME no fine tune
    }

    // --------------------------------------------
    // seasonal compositions
    // --------------------------------------------
    /**
     * creates a command that follows a path, begins intaking after a delay, and
     * ends after the intake is holding or the timeout is reached
     * 
     * @param path    path to follow, usualy a path to the station
     * @param delay   start intaking after delay(seconds)
     * @param timeout end command if intake isnt holding for this long(seconds)
     * @return
     */
    private Command station(Command path, double delay, double timeout) {
        return Commands.deadline(
                    Commands.sequence(
                        new WaitCommand(delay),
                        Commands.startRun(() -> {
                            robotContainer.getSuperstructure().setWantedState(WantedSuperState.INTAKE_CORAL);
                        }, () -> {
                        }).withTimeout(timeout)
                        .until(() -> !robotContainer.getSuperstructure().intakeBypassed && robotContainer.getSuperstructure().isHolding())
                    ),
                    path
                ).withName("follow path and station intake");
    }

    private Command alignThenScore(BranchSide side, ReefLevel level) {
        return Commands.sequence(
            Commands.parallel(
                Commands.either(
                    robotContainer.getSuperstructure().getAutoAlignLeft(),
                    robotContainer.getSuperstructure().getAutoAlignRight(),
                    () -> side == BranchSide.LEFT),
                new InstantCommand(() -> {
                    switch (level) {
                        case L1:
                            robotContainer.getSuperstructure().setWantedState(WantedSuperState.L1);
                            break;
                        case L2:
                            robotContainer.getSuperstructure().setWantedState(WantedSuperState.L2);
                            break;
                        case L3:
                            robotContainer.getSuperstructure().setWantedState(WantedSuperState.L3);
                            break;
                        case L4:
                            robotContainer.getSuperstructure().setWantedState(WantedSuperState.L4);
                            break;
                    }
                })
            ),
            score()
        ).withName("align to " + (side == BranchSide.LEFT ? "left" : "right") + " branch, then score");
    }

    private double timeout = 0.0;

    /**
     * score until superstructure stops, or after 0.5 sec if intake sensor is
     * bypassed,
     * ends with 0.5 sec delay for the affector to stow
     * 
     * @return
     */
    private Command score() {
        timeout = 0;
        return Commands.startRun(() -> {
            robotContainer.getSuperstructure().score();
        }, () -> {
            timeout += 0.02;
        }).until(() -> robotContainer.getSuperstructure().intakeBypassed ? 
                timeout >= 0.5//scoring delay if intake sensor is bypassed
                : !robotContainer.getSuperstructure().scoring
        ).andThen(
            new InstantCommand(() -> {
                robotContainer.getSuperstructure().endScore();
            }),
            new WaitCommand(0.5)//end delay
        ).withName("score");
    }

    // --------------------------------------------
    // helper functions
    // --------------------------------------------
    private PathPlannerTrajectory mergeTrajectories(PathPlannerTrajectory... in) {
        List<PathPlannerTrajectoryState> traj = new LinkedList<PathPlannerTrajectoryState>();

        double timeOffset = 0.0;
        for (int i = 0; i < in.length; i++) {
            PathPlannerTrajectory trajectory = in[i];
            PathPlannerTrajectoryState[] states = trajectory.getStates().toArray(new PathPlannerTrajectoryState[0])
                    .clone();
            double nextoffset = states[states.length - 1].timeSeconds;
            for (PathPlannerTrajectoryState s1 : states) {
                traj.add(s1.copyWithTime(timeOffset + s1.timeSeconds));
            }
            timeOffset += nextoffset;
        }
        return new PathPlannerTrajectory(traj);
    }

    private PathPlannerTrajectory getTraj(PathPlannerPath path) {
        return (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red ? path : path)
                .getIdealTrajectory(DriveConstants.PP_CONFIG).get();
    }

    private PathPlannerTrajectory[] getTraj(PathPlannerPath... paths) {
        PathPlannerTrajectory[] traj = new PathPlannerTrajectory[paths.length];
        for (int i = 0; i < paths.length; i++) {
            traj[i] = getTraj(paths[i]);
        }
        return traj;
    }
}