package frc.utils;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.constants.Constants;

public class TimerHandler {

    private static double autoStart = 0.0;
    private static double teleopStart = 0.0;

    public static void updateTeleop(){
        Logger.recordOutput("time/teleop/UpTime"       , Timer.getFPGATimestamp()-teleopStart);
        Logger.recordOutput("time/teleop/RemainingTime", ExtraMath.holdPositive(Constants.TELEOP_TIME-(Timer.getFPGATimestamp()-teleopStart)));
        Logger.recordOutput("time/RemainingTime", ExtraMath.holdPositive(Constants.TELEOP_TIME-(Timer.getFPGATimestamp()-teleopStart)));
    }
    public static void updateAuto(){
        Logger.recordOutput("time/auto/UpTime"       , Timer.getFPGATimestamp()-autoStart);
        Logger.recordOutput("time/auto/RemainingTime", ExtraMath.holdPositive(Constants.AUTO_TIME-(Timer.getFPGATimestamp()-autoStart)));
        Logger.recordOutput("time/RemainingTime", ExtraMath.holdPositive(Constants.AUTO_TIME-(Timer.getFPGATimestamp()-autoStart)));
    }
    public static void initTeleop(){
        teleopStart = Timer.getFPGATimestamp();
        Logger.recordOutput("time/teleop/StartTime", teleopStart);
    }
    public static void initAuto(){
        autoStart = Timer.getFPGATimestamp();
        Logger.recordOutput("time/auto/StartTime", autoStart);
    }
    public static void init(){
        Logger.recordOutput("time/upTime", 0.0);
        Logger.recordOutput("time/RemainingTime", 0.0);

        Logger.recordOutput("time/auto/StartTime"    , 0.0);
        Logger.recordOutput("time/auto/UpTime"       , 0.0);
        Logger.recordOutput("time/auto/RemainingTime", 0.0);
        Logger.recordOutput("time/auto/TotalTime"    , Constants.AUTO_TIME);
        
        Logger.recordOutput("time/teleop/StartTime"    , 0.0);
        Logger.recordOutput("time/teleop/UpTime"       , 0.0);
        Logger.recordOutput("time/teleop/RemainingTime", 0.0);
        Logger.recordOutput("time/teleop/TotalTime"    , Constants.TELEOP_TIME);
    }
    public static void update(){
        Logger.recordOutput("time/upTime", Timer.getFPGATimestamp());
    } 
    public static double getTeleopRemaining(){
        return ExtraMath.holdPositive(Constants.TELEOP_TIME-(Timer.getFPGATimestamp()-teleopStart));
    }
    public static double getAutoRemaining(){
        return ExtraMath.holdPositive(Constants.AUTO_TIME-(Timer.getFPGATimestamp()-autoStart));
    }
    
}
