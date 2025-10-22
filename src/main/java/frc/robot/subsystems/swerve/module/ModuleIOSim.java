package frc.robot.subsystems.swerve.module;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.Arrays;

import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.robot.constants.DriveConstants.module;
import frc.utils.BatteryVoltageSim;
import frc.utils.PID;
import frc.utils.ProfiledPID;
import frc.utils.SimpleFF;
import frc.utils.SparkUtil;

/** Physics sim implementation of module IO. */
public class ModuleIOSim implements ModuleIO {

    private final SwerveModuleSimulation moduleSim;

    private final SimulatedMotorController.GenericMotorController driveSim;
    private final SimulatedMotorController.GenericMotorController turnSim;

    private boolean driveClosedLoop = false;
    private boolean turnClosedLoop = false;

    private PID driveController = new PID(module.DRIVE_PID_SIM);
    private ProfiledPID turnController = new ProfiledPID(module.TURN_PID_SIM);

    private final SimpleFF driveFF = new SimpleFF(module.DRIVE_FF_SIM);

    private double driveAppliedVolts = 0.0;
    private double turnAppliedVolts = 0.0;

    private double turnGoal = 0.0;
    private double driveGoal = 0.0;

    private double turnPosRad = 0.0;

    public ModuleIOSim(SwerveModuleSimulation moduleSim) {
        this.moduleSim = moduleSim;
        // Create drive and turn sim models
        driveSim = moduleSim
                .useGenericMotorControllerForDrive()
                .withCurrentLimit(Amps.of(40));
        turnSim = moduleSim
                .useGenericControllerForSteer()
                .withCurrentLimit(Amps.of(40));

        // Enable wrapping for turn PID
        turnController.enableContinuousInput(module.TURN_MIN_POS, module.TURN_MAX_POS);

        BatteryVoltageSim.getInstance().addCurrentSource(() -> moduleSim.getDriveMotorSupplyCurrent().in(Amps));
        BatteryVoltageSim.getInstance().addCurrentSource(() -> moduleSim.getSteerMotorSupplyCurrent().in(Amps));
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {

        turnPosRad = MathUtil.inputModulus(moduleSim.getSteerAbsoluteFacing().getRadians(), module.TURN_MIN_POS, module.TURN_MAX_POS);
        turnGoal = MathUtil.inputModulus(turnGoal, module.TURN_MIN_POS, module.TURN_MAX_POS);

        // Run closed-loop control
        if (driveClosedLoop) {
            driveAppliedVolts = driveFF.calculate(driveGoal)
                    + driveController.calculate(moduleSim.getDriveWheelFinalSpeed().in(RadiansPerSecond), driveGoal);
        } else {
            driveController.reset();
        }
        if (turnClosedLoop) {
            turnAppliedVolts = module.TURN_FF_SIM.kS() * Math.signum(turnController.getSetpoint().position)
                    + turnController.calculate(turnPosRad, turnGoal);
        } else {
            turnController.reset(turnPosRad);
            ;
        }

        // Update simulation state
        turnSim.requestVoltage(
                Volts.of(MathUtil.clamp(turnAppliedVolts, -RoboRioSim.getVInVoltage(), RoboRioSim.getVInVoltage())));
        driveSim.requestVoltage(
                Volts.of(MathUtil.clamp(driveAppliedVolts, -RoboRioSim.getVInVoltage(), RoboRioSim.getVInVoltage())));

        // Update drive inputs
        inputs.driveConnected = true;
        inputs.drivePositionRad = moduleSim.getDriveWheelFinalPosition().in(Radians);
        inputs.driveVelocityRadPerSec = moduleSim.getDriveWheelFinalSpeed().in(RadiansPerSecond);
        inputs.driveAppliedVolts = driveAppliedVolts;
        inputs.driveCurrentAmps = Math.abs(moduleSim.getDriveMotorSupplyCurrent().in(Amps));

        // Update turn inputs
        inputs.turnConnected = true;
        inputs.turnPositionRad = turnPosRad;
        inputs.turnVelocityRadPerSec = moduleSim.getSteerAbsoluteEncoderSpeed().in(RadiansPerSecond);
        inputs.turnAppliedVolts = turnAppliedVolts;
        inputs.turnCurrentAmps = Math.abs(moduleSim.getSteerMotorSupplyCurrent().in(Amps));

        // Update odometry inputs
        inputs.odometryTimestamps = SparkUtil.getSimulationOdometryTimeStamps();
        inputs.odometryDrivePositionsRad = Arrays.stream(moduleSim.getCachedDriveWheelFinalPositions())
                .mapToDouble(angle -> angle.in(Radians))
                .toArray();
        inputs.odometryTurnPositionsRad = inputs.odometryTurnPositionsRad = Arrays
                .stream(moduleSim.getCachedSteerAbsolutePositions())
                .mapToDouble(angle -> angle.getRadians())
                .toArray();
    }

    @Override
    public void setDriveOpenLoop(double output) {
        driveClosedLoop = false;
        driveAppliedVolts = output;
    }

    @Override
    public void setTurnOpenLoop(double output) {
        turnClosedLoop = false;
        turnAppliedVolts = output;
    }

    @Override
    public void setDriveVelocity(double velocityRadPerSec) {
        driveClosedLoop = true;
        driveGoal = velocityRadPerSec;
    }

    @Override
    public void setTurnPosition(Rotation2d rotation) {
        turnClosedLoop = true;
        turnGoal = rotation.getRadians();
    }
}