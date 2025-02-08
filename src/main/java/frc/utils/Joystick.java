package frc.utils;

import edu.wpi.first.math.geometry.Translation2d;

public class Joystick {
    public static Translation2d deadzone(double deadzone, double x, double y) {
        if (ExtraMath.getMagnitude(x, y) < deadzone) {
            return new Translation2d();
        }
        return new Translation2d(x, y);
    }
}
