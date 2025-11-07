package frc.robot.subsystems.swerve.gyro;

import org.ironmaple.simulation.drivesims.GyroSimulation;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import frc.utils.SparkUtil;

public class GyroIOSim implements GyroIO {

    private final GyroSimulation gyro;

    public GyroIOSim(GyroSimulation gyro) {
        this.gyro = gyro;
    }

    public void reset(double heading) {
        gyro.setRotation(new Rotation2d(heading));
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.connected = true;
        inputs.yawPosition = gyro.getGyroReading();
        inputs.yawVelocityRadPerSec = gyro.getMeasuredAngularVelocity().baseUnitMagnitude();

        inputs.odometryYawPositions = gyro.getCachedGyroReadings();
        inputs.odometryYawTimestamps = SparkUtil.getSimulationOdometryTimeStamps();

        inputs.angle = new Rotation3d(0, 0, inputs.yawPositionRad);
    }
}