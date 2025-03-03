package frc.utils;

import java.util.function.DoubleSupplier;
import edu.wpi.first.math.geometry.Translation2d;

public class Joystick {
    public static Translation2d deadzone(double deadzone, double x, double y) {
        if (ExtraMath.getMagnitude(x, y) < deadzone) {
            return new Translation2d();
        }
        return new Translation2d(x, y);
    }

    public static class duelJoystickAxis{
        public DoubleSupplier lx = () -> 0;
        public DoubleSupplier ly = () -> 0;
        public DoubleSupplier rx = () -> 0;
        public DoubleSupplier ry = () -> 0;

        public duelJoystickAxis(DoubleSupplier lx, DoubleSupplier ly, DoubleSupplier rx, DoubleSupplier ry){
            this.lx = lx;
            this.ly = ly;
            this.rx = rx;
            this.ry = ry;
        }
        public duelJoystickAxis() {}
    }
}
