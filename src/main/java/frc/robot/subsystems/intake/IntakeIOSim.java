package frc.robot.subsystems.intake;


import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static frc.robot.constants.IntakeConstants.pivotToCoral;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnFly;
import org.littletonrobotics.junction.AutoLogOutput;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.WristConstants;
import frc.robot.subsystems.affector.Affector;

public class IntakeIOSim implements IntakeIO {
    
    private boolean holding = false;
    @AutoLogOutput
    private double coralLocation = 0;
    private SwerveDriveSimulation driveSim;
    private Affector affector;

    private double voltage = 0.0;
    
    private double vel = 0.0;

    public IntakeIOSim(SwerveDriveSimulation driveSim, Affector elevator) {
        this.driveSim = driveSim;
        this.affector = elevator;
    }
    
    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        vel = voltage * Units.rotationsPerMinuteToRadiansPerSecond(2348.9/12.0);

        // Update input values
        inputs.motorVoltage = voltage;
        inputs.motorCurrent = 0;
        inputs.motorTemperature = -1;
        inputs.motorVelocityRPM = Units.radiansPerSecondToRotationsPerMinute(vel);

        inputs.holding = holding;

        if(holding){
            coralLocation += vel * Units.inchesToMeters(1.5) * 0.02;
        }
        if(!holding && affector.getPosition().wrist < 0 && voltage > 1){
            holding = true;
            coralLocation = 0;
        }
        if(coralLocation > Units.inchesToMeters(10) && holding){
            holding = false;
            SimulatedArena.getInstance().addGamePieceProjectile(new ReefscapeCoralOnFly(
                driveSim.getSimulatedDriveTrainPose().getTranslation(),
                IntakeConstants.WRIST_POS.toTranslation2d().plus(new Translation2d(Math.cos(affector.getPosition().wrist)*pivotToCoral, 0)),
                driveSim.getDriveTrainSimulatedChassisSpeedsFieldRelative(),
                driveSim.getSimulatedDriveTrainPose().getRotation().rotateBy(Rotation2d.kCCW_90deg),
                Meters.of(affector.getPosition().elev + WristConstants.WRIST_POS.getZ() + Math.sin(affector.getPosition().wrist)*pivotToCoral),
                MetersPerSecond.of(vel * -Units.inchesToMeters(1.5)),
                Radians.of(affector.getPosition().wrist+Units.degreesToRadians(90))
            ));
        }
        if(coralLocation < -Units.inchesToMeters(10) && holding){
            holding = false;
            SimulatedArena.getInstance().addGamePieceProjectile(new ReefscapeCoralOnFly(
                driveSim.getSimulatedDriveTrainPose().getTranslation(),
                IntakeConstants.WRIST_POS.toTranslation2d().plus(new Translation2d(Math.cos(affector.getPosition().wrist)*pivotToCoral, 0)),
                driveSim.getDriveTrainSimulatedChassisSpeedsFieldRelative(),
                driveSim.getSimulatedDriveTrainPose().getRotation().rotateBy(Rotation2d.kCCW_90deg),
                Meters.of(affector.getPosition().elev + WristConstants.WRIST_POS.getZ() + Math.sin(affector.getPosition().wrist)*pivotToCoral),
                MetersPerSecond.of(vel * Units.inchesToMeters(0.5)),
                Radians.of(affector.getPosition().wrist+Units.degreesToRadians(270))
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
