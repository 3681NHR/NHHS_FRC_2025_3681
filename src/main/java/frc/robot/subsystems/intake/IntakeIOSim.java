package frc.robot.subsystems.intake;


import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static frc.robot.constants.IntakeConstants.pivotToCoral;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnFly;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.WristConstants;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.wrist.Wrist;

public class IntakeIOSim implements IntakeIO {
    
    private boolean holding = false;
    @AutoLogOutput
    private double coralLocation = 0;
    private SwerveDriveSimulation driveSim;
    private Elevator elevator;
    private Wrist wrist;

    private double voltage = 0.0;
    
    private double vel = 0.0;

    public IntakeIOSim(SwerveDriveSimulation driveSim, Elevator elevator, Wrist wrist) {
        this.driveSim = driveSim;
        this.elevator = elevator;
        this.wrist = wrist;
    }
    
    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        vel = voltage * 20;//TODO tune

        // Update input values
        inputs.motorVoltage = voltage;
        inputs.motorCurrent = 0;
        inputs.motorTemperature = -1;
        inputs.motorVelocityRPM = Units.radiansPerSecondToRotationsPerMinute(vel);

        inputs.holding = holding;

        if(holding){
            coralLocation += vel * Units.inchesToMeters(1.5) * 0.02;
        }
        if(!holding && wrist.getPos() < 0 && voltage > 1){
            holding = true;
            coralLocation = 0;
        }
        if(coralLocation > Units.inchesToMeters(10) && holding){
            holding = false;
            SimulatedArena.getInstance().addGamePieceProjectile(new ReefscapeCoralOnFly(
                driveSim.getSimulatedDriveTrainPose().getTranslation(),
                IntakeConstants.WRIST_POS.toTranslation2d().plus(new Translation2d(Math.cos(wrist.getPos())*pivotToCoral, 0)),
                driveSim.getDriveTrainSimulatedChassisSpeedsFieldRelative(),
                driveSim.getSimulatedDriveTrainPose().getRotation().rotateBy(Rotation2d.kCCW_90deg),
                Meters.of(elevator.getPosition() + WristConstants.WRIST_POS.getZ() + Math.sin(wrist.getPos())*pivotToCoral),
                MetersPerSecond.of(vel * Units.inchesToMeters(1.5)),
                Radians.of(wrist.getPos()-Units.degreesToRadians(90))
            ));
        }
    }
    
    @Override
    public void setVoltage(double voltage) {
        this.voltage = voltage;
    }
    
    
    @Override
    public void setNeutralMode(boolean brake) {
    }
    
}
