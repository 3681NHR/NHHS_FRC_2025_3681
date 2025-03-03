package frc.robot.subsystems.elevator;

import static frc.robot.constants.ElevatorConstants.MAX_POS;
import static frc.robot.constants.ElevatorConstants.MIN_POS;
import static frc.robot.constants.ElevatorConstants.POS_TOLERANCE;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {

    private ElevatorIO io;
    private ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
    @AutoLogOutput
    private double posSet = 0.0;

    @AutoLogOutput
    private boolean homed = false;
    @AutoLogOutput
    private boolean openloop = false;

    private Alert notHomed = new Alert("elevator is not homed", AlertType.kWarning);
    private Alert noLim = new Alert("elevator limits not enforced", AlertType.kWarning);

    private LoggedNetworkBoolean limits = new LoggedNetworkBoolean("overrides/elevatorLimits", true);

    @AutoLogOutput
    private boolean brake = true;
    
    public Elevator(ElevatorIO io){
        this.io = io;

    }

    @Override
    public void periodic(){
        io.updateInputs(inputs);
        Logger.processInputs("Elevator", inputs);

        notHomed.set(!homed);

        if(!openloop){
            if(!homed && inputs.elevatorPositionMeters < 0){
                io.resetElevatorPosition(0);
            }
            if(homed && limits.get()){
                posSet = MathUtil.clamp(posSet, MIN_POS, MAX_POS);
            }

            io.setElevatorTargetLocation(posSet);
        } else {
            posSet = inputs.elevatorPositionMeters;
        }

        noLim.set(!homed || openloop || !limits.get());

        Logger.recordOutput("poses/innerStage(2)", new Pose3d(new Translation3d(0, 0, getPosition()), new Rotation3d()));
        Logger.recordOutput("poses/middleStage(1)", new Pose3d(new Translation3d(0, 0, getPosition()*0.544561), new Rotation3d()));
    }

    public void setHomed(boolean homed){
        this.homed = homed;
    }
    public boolean isHomed(){
        return homed;
    }

    public void setVoltage(double voltage){
        io.moveElevatorOpenLoop(voltage);
        openloop = true;
    }

    public void setTargetPos(double pos){
        this.posSet = pos;
        openloop = false;
    }
    public double getPositionSet(){
        return posSet;
    }
    public double getPosition(){
        return inputs.elevatorPositionMeters;
    }
    public double getVelocity() {
        return inputs.elevatorVelocityMetersPerSec;
    }
    public void resetPos(double pos){
        io.resetElevatorPosition(pos);
    }

    public Command man(DoubleSupplier change){
        return run(() -> {
            posSet += change.getAsDouble();
        }).beforeStarting(() -> {
            //reset pos on start to avoid jumping
            posSet = inputs.elevatorPositionMeters;
        });
    }

    public void toggleBrake(){
        io.setElevatorNeutralMode(brake);
        brake = !brake;
    }
    public void setBrake(boolean brake){
        io.setElevatorNeutralMode(brake);
        this.brake = brake;
    }
    
    public boolean inPosition(){
        return Math.abs(inputs.elevatorPositionMeters - posSet) < POS_TOLERANCE;
    }
}
