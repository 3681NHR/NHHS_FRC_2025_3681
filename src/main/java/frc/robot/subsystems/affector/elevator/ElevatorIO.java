package frc.robot.subsystems.affector.elevator;

import org.littletonrobotics.junction.AutoLog;

/**
 * interface for elevator IO
 * <p>
 * all hardware interface will be done through this interface, control is done
 * through commands and inputs are sent through the inputs class
 * 
 */
public interface ElevatorIO {

    /**
     * update all inputs from the hardware, functions as a periodic method
     */
    public default void updateInputs(ElevatorIOInputs inputs) {
    }

    /**
     * set voltage for both motors
     * 
     * @param voltage voltage to set motors to in volts
     */
    public default void setVoltage(double voltage) {
    }

    /**
     * set brake mode for both motors
     * ,
     * <p>
     * when brake is enabled, motors will have much more resistance to being moved
     * when not driven
     * 
     * @param brake if true, brake mode is enabled
     */
    public default void setBrake(boolean brake) {
    }

    /**
     * reset the sensor readings to a given value
     * 
     * @param posMeters position to set the elevator to
     */
    public default void resetPos(double posMeters) {
    }

    /**
     * object that stores all sensor inputs, all vcalues are logged automatically
     */
    @AutoLog
    public class ElevatorIOInputs {
        public double pos = 0;
        public double vel = 0;

        public double motor1CurrentAmps = 0;
        public double motor1Voltage = 0;
        public double motor1TempC = 0;

        public double motor2CurrentAmps = 0;
        public double motor2Voltage = 0;
        public double motor2TempC = 0;
    }
}
