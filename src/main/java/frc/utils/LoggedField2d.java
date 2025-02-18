package frc.utils;

import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NTSendable;
import edu.wpi.first.networktables.NTSendableBuilder;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.util.sendable.SendableRegistry;

public class LoggedField2d implements NTSendable, AutoCloseable {
  /** Constructor. */
  @SuppressWarnings("this-escape")
  public LoggedField2d() {
    LoggedFieldObject2d obj = new LoggedFieldObject2d("Robot");
    obj.setPose(Pose2d.kZero);
    m_objects.add(obj);
    SendableRegistry.add(this, "Field");
  }

  @Override
  public void close() {
    for (LoggedFieldObject2d obj : m_objects) {
      obj.close();
    }
  }

  /**
   * Set the robot pose from a Pose object.
   *
   * @param pose 2D pose
   */
  public synchronized void setRobotPose(Pose2d pose) {
    m_objects.get(0).setPose(pose);
  }

  /**
   * Set the robot pose from x, y, and rotation.
   *
   * @param xMeters X location, in meters
   * @param yMeters Y location, in meters
   * @param rotation rotation
   */
  public synchronized void setRobotPose(double xMeters, double yMeters, Rotation2d rotation) {
    m_objects.get(0).setPose(xMeters, yMeters, rotation);
  }

  /**
   * Get the robot pose.
   *
   * @return 2D pose
   */
  public synchronized Pose2d getRobotPose() {
    return m_objects.get(0).getPose();
  }

  /**
   * Get or create a field object.
   *
   * @param name The field object's name.
   * @return Field object
   */
  public synchronized LoggedFieldObject2d getObject(String name) {
    for (LoggedFieldObject2d obj : m_objects) {
      if (obj.m_name.equals(name)) {
        return obj;
      }
    }
    LoggedFieldObject2d obj = new LoggedFieldObject2d(name);
    m_objects.add(obj);
    if (m_table != null) {
      synchronized (obj) {
        obj.m_entry = m_table.getDoubleArrayTopic(name).getEntry(new double[] {});
      }
    }
    return obj;
  }

  /**
   * Get the robot object.
   *
   * @return Field object for robot
   */
  public synchronized LoggedFieldObject2d getRobotObject() {
    return m_objects.get(0);
  }

  @Override
  public void initSendable(NTSendableBuilder builder) {
    builder.setSmartDashboardType("Field2d");

    synchronized (this) {
      m_table = builder.getTable();
      for (LoggedFieldObject2d obj : m_objects) {
        synchronized (obj) {
          obj.m_entry = m_table.getDoubleArrayTopic(obj.m_name).getEntry(new double[] {});
          obj.updateEntry(true);
        }
      }
    }
  }

  private NetworkTable m_table;
  private final List<LoggedFieldObject2d> m_objects = new ArrayList<>();

    public void Log(String key){
        for(LoggedFieldObject2d obj : m_objects){
            Logger.recordOutput(key+"/"+obj.m_name, obj.getPoses().stream().toArray(Pose2d[]::new));
        }
        
    }
}
