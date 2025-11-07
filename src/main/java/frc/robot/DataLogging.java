package frc.robot;

// Forked from FRC Team 2832 "The Livonia Warriors"

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** The DataLogging class contains all the logic for using telemetry. */
public class DataLogging {

  private DoubleLogEntry loopTime;
  private double startTime;
  private boolean everBrownout = false;

  private DataLogging() {
    // Starts recording to data log
    DataLogManager.start();
    final DataLog log = DataLogManager.getLog();

    DataLogManager.log(String.format("Brownout Voltage: %f", RobotController.getBrownoutVoltage()));

    loopTime = new DoubleLogEntry(log, "/robot/LoopTime");
  }

  private static class InstanceHolder {
    private static final DataLogging instance = new DataLogging();
  }

  /**
   * Gets the datalogging Singleton object.
   *
   * @return DataLogging
   */
  public static DataLogging getInstance() {
    return InstanceHolder.instance;
  }

  /**
   * Runs at each loop slice.. This method should be called in the robotPeriodic method in
   * Robot.java. the code must be the last thing in the method.
   *
   * <pre>{@code
   * //must be at end
   * datalog.periodic();
   * }</pre>
   */
  public void periodic() {

    if (RobotController.isBrownedOut()) {
      everBrownout = true;
    }

    SmartDashboard.putNumber("Batt Volt", RobotController.getBatteryVoltage());
    SmartDashboard.putBoolean("Brown Out", RobotController.isBrownedOut());
    SmartDashboard.putBoolean("Ever Browned Out", getEverBrownOut());

    if (Constants.LOOP_TIMING_LOG) {
      loopTime.append(Timer.getFPGATimestamp() - startTime);
    }
  }

  /**
   * Called from robot.java immediately after the robotContainer is created.
   *
   * @param robotContainer The robotContainer just constructed.
   */
  public void dataLogRobotContainerInit(RobotContainer robotContainer) {

    PowerDistribution pdp = robotContainer.getPdp();
    SmartDashboard.putData("PDP", pdp);
  }

  public void startLoopTime() {
    startTime = Timer.getFPGATimestamp();
  }

  public final boolean getEverBrownOut() {
    return this.everBrownout;
  }
}
