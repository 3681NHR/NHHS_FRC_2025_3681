package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Superstructure extends SubsystemBase{
    

    
    public enum WantedSuperState {
        HOME,
        STOPPED,
        DEFAULT_STATE,
        // FORCE_RELOCALIZE_LEFT,
        // FORCE_RELOCALIZE_RIGHT,
        INTAKE_CORAL_FROM_STATION_STRAIGHT,
        // SCORE_L1,
        // SCORE_LEFT_L2,
        // SCORE_LEFT_L3,
        // SCORE_LEFT_L4,
        // SCORE_RIGHT_L2,
        // SCORE_RIGHT_L3,
        // SCORE_RIGHT_L4,
        MANUAL_L4,
        MANUAL_L3,
        MANUAL_L2,
        MANUAL_L1,
        // CLIMB
    }

    public enum CurrentSuperState {
        HOME,
        STOPPED,
        NO_PIECE_TELEOP,
        HOLDING_CORAL_TELEOP,
        NO_PIECE_AUTO,
        HOLDING_CORAL_AUTO,
        // FORCE_RELOCALIZE_LEFT,
        // FORCE_RELOCALIZE_RIGHT,
        INTAKE_CORAL_FROM_STATION,
        // SCORE_TELEOP_L1_MANUAL_ALIGNMENT,
        // SCORE_L1,
        // SCORE_LEFT_TELEOP_L2,
        // SCORE_LEFT_TELEOP_L3,
        // SCORE_LEFT_TELEOP_L4,
        // SCORE_RIGHT_TELEOP_L2,
        // SCORE_RIGHT_TELEOP_L3,
        // SCORE_RIGHT_TELEOP_L4,
        // SCORE_AUTO_L1,
        // SCORE_LEFT_AUTO_L2,
        // SCORE_LEFT_AUTO_L3,
        // SCORE_LEFT_AUTO_L4,
        // SCORE_RIGHT_AUTO_L2,
        // SCORE_RIGHT_AUTO_L3,
        // SCORE_RIGHT_AUTO_L4,
        MANUAL_L4,
        MANUAL_L3,
        MANUAL_L2,
        MANUAL_L1,
        // CLIMB
    }


}
