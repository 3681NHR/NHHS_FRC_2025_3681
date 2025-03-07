package frc.utils;

public class PIDGains {
    /**
     * simple record for PID gains
     */
    public static record PID(double kP, double kI, double kD) {
        public PID {
            if (kP < 0 || kI < 0 || kD < 0) {
                throw new IllegalArgumentException("PID values must be positive");
            }
        }
    }
    /**
     * simple record for PID gains, includes max speed and max acceleration
     */
    public static record ProfiledPID(double kP, double kI, double kD, double maxSpeed, double maxAccel) {
        public ProfiledPID {
            if (kP < 0 || kI < 0 || kD < 0) {
                throw new IllegalArgumentException("PID values must be positive");
            }
        }
    }
    /**
     * simple record for feedforward gains
     */
    public static record SimpleFF(double kS, double kV, double kA) {
        public SimpleFF {
            if (kS < 0 || kV < 0 || kA < 0) {
                throw new IllegalArgumentException("FF values must be positive");
            }
        }
    }
    /**
     * simple record for feedforward gains, includes kG for elevator and arm ff
     */
    public static record GravityFF(double kS, double kG, double kV, double kA) {
        public GravityFF {
            if (kS < 0 || kV < 0 || kA < 0) {
                throw new IllegalArgumentException("FF values must be positive");
            }
        }
    }
}
