package frc.robot.nt;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;

/**
 * Subscribes to Nav/cmd on NT (published by the Orange Pi nt_bridge from ROS2 /cmd_vel).
 *
 * Watchdog rules (matches the Pi side):
 *  - if {@code heartbeat} hasn't incremented for {@link NtContract#NAV_CMD_MAX_AGE_SEC},
 *    {@link #isFresh()} returns false and {@link #vx()}/{@link #vy()}/{@link #omega()}
 *    all return 0, so the drive stops.
 *  - the Pi bridge already publishes (0,0,0) on its own watchdog timeout, but the rio
 *    enforces its own here too.
 */
public class NtCmdVelReceiver {
    private final DoubleSubscriber vxSub;
    private final DoubleSubscriber vySub;
    private final DoubleSubscriber omegaSub;
    private final IntegerSubscriber heartbeatSub;
    private final DoubleSubscriber timestampSub;

    private long lastHeartbeat = -1L;
    private double lastHeartbeatChangeFpga = 0.0;

    public NtCmdVelReceiver() {
        this(NetworkTableInstance.getDefault());
    }

    public NtCmdVelReceiver(NetworkTableInstance inst) {
        NetworkTable table = inst.getTable(NtContract.NAV_CMD_TABLE);
        vxSub = table.getDoubleTopic(NtContract.NAV_CMD_VX).subscribe(0.0);
        vySub = table.getDoubleTopic(NtContract.NAV_CMD_VY).subscribe(0.0);
        omegaSub = table.getDoubleTopic(NtContract.NAV_CMD_OMEGA).subscribe(0.0);
        heartbeatSub = table.getIntegerTopic(NtContract.NAV_CMD_HEARTBEAT).subscribe(-1L);
        timestampSub = table.getDoubleTopic(NtContract.NAV_CMD_TIMESTAMP).subscribe(0.0);
    }

    /** Call once per loop to update watchdog state from latest heartbeat. */
    public void poll() {
        long hb = heartbeatSub.get();
        if (hb != lastHeartbeat) {
            lastHeartbeat = hb;
            lastHeartbeatChangeFpga = Timer.getFPGATimestamp();
        }
    }

    public boolean isFresh() {
        return Timer.getFPGATimestamp() - lastHeartbeatChangeFpga < NtContract.NAV_CMD_MAX_AGE_SEC;
    }

    public double vx() { return isFresh() ? vxSub.get() : 0.0; }
    public double vy() { return isFresh() ? vySub.get() : 0.0; }
    public double omega() { return isFresh() ? omegaSub.get() : 0.0; }

    public long heartbeat() { return lastHeartbeat; }
    public double piTimestamp() { return timestampSub.get(); }
}
