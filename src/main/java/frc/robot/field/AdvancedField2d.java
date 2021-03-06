package frc.robot.field;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NTSendable;
import edu.wpi.first.networktables.NTSendableBuilder;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.util.sendable.SendableRegistry;
import java.util.ArrayList;
import java.util.List;

/**
 * 2D representation of game field for dashboards.
 *
 * <p>An object's pose is the location shown on the dashboard view. Note that for the robot, this
 * may or may not match the internal odometry. For example, the robot is shown at a particular
 * starting location, the pose in this class would represent the actual location on the field, but
 * the robot's internal state might have a 0,0,0 pose (unless it's initialized to something
 * different).
 *
 * <p>As the user is able to edit the pose, code performing updates should get the robot pose,
 * transform it as appropriate (e.g. based on wheel odometry), and set the new pose.
 *
 * <p>This class provides methods to set the robot pose, but other objects can also be shown by
 * using the getObject() function. Other objects can also have multiple poses (which will show the
 * object at multiple locations).
 */
public class AdvancedField2d implements NTSendable {
  /** Constructor. */
  public AdvancedField2d() {
    FieldObject obj = new FieldObject("Robot");
    obj.setPose(new Pose2d());
    m_objects.add(obj);
    SendableRegistry.add(this, "Advanced Field");
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
  @SuppressWarnings("ParameterName")
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
  public synchronized FieldObject getObject(String name) {
    for (FieldObject obj : m_objects) {
      if (obj.m_name.equals(name)) {
        return obj;
      }
    }
    FieldObject obj = new FieldObject(name);
    m_objects.add(obj);
    if (m_table != null) {
      synchronized (obj) {
        obj.m_entry = m_table.getEntry(name);
      }
    }
    return obj;
  }

  /**
   * Get the robot object.
   *
   * @return Field object for robot
   */
  public synchronized FieldObject getRobotObject() {
    return m_objects.get(0);
  }

  @Override
  public void initSendable(NTSendableBuilder builder) {
    builder.setSmartDashboardType("AdvancedField2d");

    synchronized (this) {
      m_table = builder.getTable();
      for (FieldObject obj : m_objects) {
        synchronized (obj) {
          obj.m_entry = m_table.getEntry(obj.m_name);
          obj.updateEntry(true);
        }
      }
    }
  }

  private NetworkTable m_table;
  private final List<FieldObject> m_objects = new ArrayList<>();
}
