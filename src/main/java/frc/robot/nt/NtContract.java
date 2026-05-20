package frc.robot.nt;

/**
 * Table + key names shared with the Orange Pi nt_bridge.
 * Must match src/nt_bridge/config/nt_bridge.yaml on the Pi.
 */
public final class NtContract {
    private NtContract() {}

    // Pi -> Rio
    public static final String NAV_CMD_TABLE = "Nav/cmd";
    public static final String NAV_CMD_VX = "vx";
    public static final String NAV_CMD_VY = "vy";
    public static final String NAV_CMD_OMEGA = "omega";
    public static final String NAV_CMD_HEARTBEAT = "heartbeat";
    public static final String NAV_CMD_TIMESTAMP = "timestamp";

    public static final String PI_PRESS_BUTTON_KEY = "Pi/elevator/press_button";
    /** Pi-issued mission command: "load", "unload", "stow", etc. */
    public static final String PI_MISSION_CMD_KEY = "Pi/mission/cmd";

    // Rio -> Pi
    public static final String ROBOT_ODOM_TABLE = "Robot/odom";
    public static final String ODOM_X = "x";
    public static final String ODOM_Y = "y";
    public static final String ODOM_THETA = "theta";
    public static final String ODOM_VX = "vx";
    public static final String ODOM_VY = "vy";
    public static final String ODOM_OMEGA = "omega";

    public static final String ROBOT_IMU_TABLE = "Robot/imu";
    public static final String IMU_YAW = "yaw";
    public static final String IMU_YAW_RATE = "yaw_rate";
    public static final String IMU_ACCEL_X = "accel_x";
    public static final String IMU_ACCEL_Y = "accel_y";
    public static final String IMU_ACCEL_Z = "accel_z";

    public static final String ROBOT_PRESS_DONE_KEY = "Robot/elevator/press_done";
    public static final String ROBOT_MISSION_START_KEY = "Robot/mission/start_pressed";
    public static final String ROBOT_MISSION_LOADED_KEY = "Robot/mission/loaded";
    public static final String ROBOT_MISSION_UNLOADED_KEY = "Robot/mission/unloaded";
    /** Held-true while the operator is holding the dead-man enable button. Not pulsed. */
    public static final String ROBOT_ENABLE_KEY = "Robot/mission/enable";
    /** Pulsed when the operator hits the restart/abort button. */
    public static final String ROBOT_RESTART_KEY = "Robot/mission/restart_pressed";

    // Watchdog: max age of last heartbeat tick (s). Matches nt_bridge's cmd_vel_max_age default.
    public static final double NAV_CMD_MAX_AGE_SEC = 0.5;

    // Notifier rate for IMU publish. Pi bridge polls at 100 Hz; we publish at the same.
    public static final double IMU_PUBLISH_HZ = 100.0;
}
