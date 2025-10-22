package frc.utils;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.util.Color;

import java.lang.Math;
import java.util.ArrayList;

public final class ExtraMath {

    public static double roundToPoint(double val, int point) {
        return (int)(val*Math.pow(10, point)) / Math.pow(10, point);
    }

    public static Rotation2d getAngle(double u, double v) {
        return new Rotation2d(Math.atan2(v, u));
    }

    public static double getMagnitude(double x, double y) {
        return Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
    }

    /**
     * get tilt of robot
     * 
     * @param angle
     * @return double[2], 0 is yaw angle(-pi to pi), 1 is tilt angle
     */
    public static double[] getTip(Rotation3d angle) {
        double[] out = new double[2];

        out[0] = Math.atan2(-angle.getX(), -angle.getY());
        out[1] = Math.hypot(angle.getX(), angle.getY());

        return out;
    }

    /**
     * process input value
     * 
     * curve function graphed here
     * {@link https://www.desmos.com/calculator/fjuc4iqjqt}
     * 
     * @param val        - number to process
     * @param multiplier - multiplier for input, mainly used for inverting
     * @param square     - polynomial curve value, roughly y=x^s
     *                   {@link https://docs.wpilib.org/en/stable/docs/software/hardware-apis/motors/wpi-drive-classes.html#squaring-inputs}
     * @param deadZone   - deadzone for input
     *                   {@link https://docs.wpilib.org/en/stable/docs/software/hardware-apis/motors/wpi-drive-classes.html#input-deadband}
     * @return
     */
    public static double processInput(Double val, Double multiplier, Double square, Double deadZone) {
        double out = val;

        if (square != null) {
            if (deadZone != null && deadZone > 0) {
                out = Math.signum(val)
                        * Math.pow((1 / (-deadZone + 1)) * Math.abs(val) - (deadZone / (-deadZone + 1)), square);
            } else {
                out = Math.signum(val) * Math.pow(Math.abs(val), square);
            }
        }
        if (deadZone != null) {
            out = MathUtil.applyDeadband(out, deadZone);
        }
        if (multiplier != null) {
            out *= multiplier;
        }

        return out;
    }

    public static double remap(double val, double inMin, double inMax, double outMin, double outMax) {
        return ((val - inMin) / (inMax - inMin) * (outMax - outMin)) + outMin;
    }

    public static double lerp(double start, double end, double val) {
        return (end - start) * val + start;
    }
    public static Color colLerp(Color start, Color end, double val) {
        // Color hsv = rgbToHsv(start);
        // Color hsvEnd = rgbToHsv(end);
        // double h = lerp(hsv.red, hsvEnd.red, val);
        // double s = lerp(hsv.green, hsvEnd.green, val);
        // double v = lerp(hsv.blue, hsvEnd.blue, val);
        // return Color.fromHSV((int)(h*180), (int)(s*255), (int)(v*255));

        double r = lerp(start.red, end.red, val);
        double g = lerp(start.green, end.green, val);
        double b = lerp(start.blue, end.blue, val);
        return new Color(r, g, b);
    }

    public static Color rgbToHsv(Color in) {
        float[] hsv = new float[3];
        java.awt.Color.RGBtoHSB((int)(in.red*255), (int)(in.green*255), (int)(in.blue*255), hsv);
        return new Color(hsv[0], hsv[1], hsv[2]);
    }

    public static double holdPositive(double in) {
        return in < 0 ? 0 : in;
    }

    public static Pose2d getNearestPose(Pose2d[] poses, Pose2d current) {
        double min = Double.MAX_VALUE;
        Pose2d out = poses[0];
        for (Pose2d pose : poses) {
            double dist = current.getTranslation().getDistance(pose.getTranslation());
            dist += Math.abs(current.getRotation().minus(pose.getRotation()).getRadians());
            dist = Math.abs(dist / 2.0);
            if (dist < min) {
                min = dist;
                out = pose;
            }
        }
        return out;
    }

    public static double getDistance(Pose2d a, Pose2d b) {
        return a.getTranslation().getDistance(b.getTranslation());
    }

    public static boolean PoseWithinTolerance(Pose2d a, Pose2d b, double toleranceLinear, double toleranceAngular) {

        double dist = getDistance(a, b);
        double angleDiff = Math.abs(a.getRotation().minus(b.getRotation()).getRadians());

        return dist < toleranceLinear && angleDiff < toleranceAngular;
    }

    public static Color normalizeCol(Color in) {
        double brt = in.red / 255.0 + in.green / 255.0 + in.blue / 255.0;
        return new Color(in.red / brt, in.green / brt, in.blue / brt);
    }

    /**
     * Derrivitive class
     */
    public static class Derrivitive {

        private double value;
        private double oldValue;
        private boolean init = false;

        /**
         * constructs a new Derrivitive object, first update will be based off initial
         * mesurement
         * 
         * @param initMesure initial mesurement
         */
        public Derrivitive(double initMesure) {
            oldValue = initMesure;
            init = true;
        }

        /**
         * constructs a new Derrivitive object, first update will return 0
         */
        public Derrivitive() {
            init = false;
        }

        /**
         * calculate the rate of change of a mesurement
         * 
         * @param mesurement current mesurement
         * @param dt         time since last update
         * @return rate of change
         */
        public double calculate(double mesurement, double dt) {
            value = mesurement;
            if (!init) {
                oldValue = value;
                init = true;
            }
            double out = (value - oldValue) / dt;
            oldValue = value;
            return out;
        }

        public void reset(double initMesure) {
            oldValue = initMesure;
        }
    }

    public static class MovingAverageFilter {
        private ArrayList<Double> window = new ArrayList<>();
        private int taps;

        public MovingAverageFilter(int taps) {
            this.taps = taps;
        }

        public double calculate(double in) {
            window.add(in);
            if (window.size() > taps) {
                window.remove(0);
            }
            double t = 0;
            for (double x : window) {
                t += x;
            }

            t = t / window.size();

            return t;
        }

        public void reset() {
            window.clear();
        }
    }
}
