package frc.robot.subsystems.elevator;

import static frc.robot.constants.ElevatorConstants.MAX_POS;
import static frc.robot.constants.ElevatorConstants.MIN_POS;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {

    private ElevatorIO io;
    private ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
    @AutoLogOutput
    private double pos = 0.0;

    @AutoLogOutput
    private boolean homed = false;
    @AutoLogOutput
    private boolean openloop = false;

    private Alert notHomed = new Alert("elevator is not homed, limits not enforced", AlertType.kWarning);

    @AutoLogOutput
    private boolean brake = true;
    
    public Elevator(ElevatorIO io){
        this.io = io;

    }

    @Override
    public void periodic(){
        io.updateInputs(inputs);
        Logger.processInputs("elevator", inputs);

        notHomed.set(!homed);

        if(!openloop){
            if(!homed && inputs.elevatorPositionMeters < 0){
                io.resetElevatorPosition(0);
            }
            pos = MathUtil.clamp(pos, MIN_POS, MAX_POS);
                
            io.setElevatorTargetLocation(pos);
        } else {
            pos = inputs.elevatorPositionMeters;
        }
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
        this.pos = pos;
        openloop = false;
    }
    public void setTargetPos(AffectorPosition pos){
        this.pos = pos.elev;
        openloop = false;
    }
    public double getPositionSet(){
        return pos;
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
            pos += change.getAsDouble();
        }).beforeStarting(() -> {
            //reset pos on start to avoid jumping
            pos = inputs.elevatorPositionMeters;
        });
    }

    public void toggleBrake(){
        io.setElevatorNeutralMode(brake);
        brake = !brake;
    }
}
