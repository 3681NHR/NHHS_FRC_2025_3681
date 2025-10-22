package frc.robot.autos;

import java.util.List;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.utils.ExtraMath;
import frc.utils.LoggedField2d;
import frc.utils.Alert.AlertType;
import frc.utils.Alert.SendableAlerts;

/**
 * A {@link edu.wpi.first.wpilibj.smartdashboard.SendableChooser} for selecting an autonomous program.
 * <br/><br/>
 * <p>
 * How to add a new autonomous program.
 *     <ol>
 *         <li>
 *             Add a new value to {@link Auto}<br/>
 *             The name of the value should use screaming snake case
 *             </li>
 *             <li>
 *                 Add a new method in {@link AutoFactory} that returns a {@link edu.wpi.first.wpilibj2.command.Command}.
 *             </li>
 *             <li>
 *                 Add a new {@link AutoProgram} to {@link AutoChooser#AUTO_PROGRAMS}.
 *             </li>
 *             <li>
 *                 Implement the autonomous program factory method.
 *             </li>
 *             <li>
 *                 Test, test, and test some more.
 *             </li>
 *         </ol>
 * </p>
 */
public class AutoChooser {

    private AutoFactory factory;
    private RobotContainer container;

    private LoggedField2d field = new LoggedField2d();
    private LoggedDashboardChooser<AutoProgram> chooser;
    private LoggedNetworkNumber timeSelector = new LoggedNetworkNumber("auto/time", 0.0);

    private final List<AutoProgram> AUTO_PROGRAMS = List.of(
        // new AutoProgram("example", AutoFactory::createExampleAuto),
            new AutoProgram("idle",  AutoFactory::createIdleAuto),
            new AutoProgram("test",  AutoFactory::createTestAuto),
            new AutoProgram("right 5", AutoFactory::createR5Auto),
            new AutoProgram("E", AutoFactory::createEAuto)
    );


    public AutoChooser(RobotContainer container){
        chooser = new LoggedDashboardChooser<>("Auto Program");
        for (int i=0; i<AUTO_PROGRAMS.size(); i++){
            if(i == 0){
                chooser.addDefaultOption(AUTO_PROGRAMS.get(i).getLabel(), AUTO_PROGRAMS.get(i));
            } else {
                chooser.addOption(AUTO_PROGRAMS.get(i).getLabel(), AUTO_PROGRAMS.get(i));
            }
        }
        this.factory = new AutoFactory(container);
        this.container = container;

        for(AutoProgram program : AUTO_PROGRAMS){
            program.update(factory);
        }
        
        SmartDashboard.putData("auto/autoPath", field);
    }

    public Command getSelected(){
        return chooser.get().getCommand(factory);
    }

    public void update(){
        if(!DriverStation.isEnabled()){
            double time = timeSelector.get() * chooser.get().getPathLength(factory);

            field.getObject("traj").setPoses(chooser.get().getPoses(factory));
            field.setRobotPose(container.getDrive().getPose());
            field.getObject("start").setPose(chooser.get().getPoseAtTime(factory, time));
            
            Logger.recordOutput("auto/selected time", ExtraMath.roundToPoint(time, 3));
            Logger.recordOutput("auto/total time"   , ExtraMath.roundToPoint(chooser.get().getPathLength(factory), 3));
        }

        Logger.recordOutput("auto/list/Auto selected", chooser.get() != null);
        Logger.recordOutput("auto/list/Robot in position", ExtraMath.PoseWithinTolerance(container.getDrive().getPose(), chooser.get().getStartingPose(factory), 0.5, Math.toRadians(20)));
        Logger.recordOutput("auto/list/FMS connected", DriverStation.isFMSAttached());
        Logger.recordOutput("auto/list/Joysticks connected", DriverStation.isJoystickConnected(0) && DriverStation.isJoystickConnected(1));
        Logger.recordOutput("auto/list/No alerts", 
                                            SendableAlerts.forGroup("Alerts").getStrings(AlertType.kError).length == 0 &&
                                            SendableAlerts.forGroup("Alerts").getStrings(AlertType.kWarning).length == 0 &&
                                            SendableAlerts.forGroup("Alerts").getStrings(AlertType.kInfo).length == 0);
        Logger.recordOutput("auto/list/Auto set", "set correct auto program");
        Logger.recordOutput("auto/list/Odometry correct", "confirm odometry is same with real position");
        Logger.recordOutput("auto/list/Controller ports", "confirm controllers are connected to right ports");
        Logger.recordOutput("auto/list/DS secure", "confirm DS is securely attached to shelf");
        // Logger.recordOutput("auto/list/", );
        // Logger.recordOutput("auto/list/Gamepiece loaded", );
    }
}