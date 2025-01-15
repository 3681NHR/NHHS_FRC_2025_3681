package frc.robot.subsystems;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {
    
    private ProfiledPIDController anglePID = new ProfiledPIDController(Constants.arm.gains.ANGLE_KP,
                                                                        Constants.arm.gains.ANGLE_KI,
                                                                        Constants.arm.gains.ANGLE_KD,
                                                                        null);
    private ArmFeedforward angleFF = new ArmFeedforward(Constants.arm.gains.ANGLE_KS,
                                                        Constants.arm.gains.ANGLE_KG,
                                                        Constants.arm.gains.ANGLE_KV);
    public ArmSubsystem(){

    }

    @Override
    public void periodic() {
    
    }
}
