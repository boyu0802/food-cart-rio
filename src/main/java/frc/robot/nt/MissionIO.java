package frc.robot.nt;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.wpilibj.Timer;

/**
 * Mission-channel half of the nt_bridge contract:
 *   in  : Pi/elevator/press_button (String)
 *   in  : Pi/mission/cmd (String)  — "load" | "unload" | "stow"
 *   out : Robot/elevator/press_done (bool, pulsed)
 *   out : Robot/mission/{start_pressed, loaded, unloaded, restart_pressed} (bool, pulsed)
 *   out : Robot/mission/enable (bool, *held* — true while dead-man button is held)
 *
 * Pi reads pulsed bools edge-triggered. {@link PulseState} holds true for ~100 ms.
 */
public class MissionIO {
    public static final double PULSE_HOLD_SEC = 0.1;

    private final StringSubscriber pressButtonSub;
    private final StringSubscriber missionCmdSub;

    private final BooleanPublisher pressDonePub;
    private final BooleanPublisher missionStartPub;
    private final BooleanPublisher missionLoadedPub;
    private final BooleanPublisher missionUnloadedPub;
    private final BooleanPublisher missionRestartPub;
    private final BooleanPublisher enablePub;

    private final PulseState pressDone = new PulseState();
    private final PulseState missionStart = new PulseState();
    private final PulseState missionLoaded = new PulseState();
    private final PulseState missionUnloaded = new PulseState();
    private final PulseState missionRestart = new PulseState();

    private String lastButton = "";
    private String lastMissionCmd = "";
    private boolean enabled = false;

    public MissionIO() {
        this(NetworkTableInstance.getDefault());
    }

    public MissionIO(NetworkTableInstance inst) {
        pressButtonSub = inst.getStringTopic(NtContract.PI_PRESS_BUTTON_KEY).subscribe("");
        missionCmdSub = inst.getStringTopic(NtContract.PI_MISSION_CMD_KEY).subscribe("");

        pressDonePub = inst.getBooleanTopic(NtContract.ROBOT_PRESS_DONE_KEY).publish();
        missionStartPub = inst.getBooleanTopic(NtContract.ROBOT_MISSION_START_KEY).publish();
        missionLoadedPub = inst.getBooleanTopic(NtContract.ROBOT_MISSION_LOADED_KEY).publish();
        missionUnloadedPub = inst.getBooleanTopic(NtContract.ROBOT_MISSION_UNLOADED_KEY).publish();
        missionRestartPub = inst.getBooleanTopic(NtContract.ROBOT_RESTART_KEY).publish();
        enablePub = inst.getBooleanTopic(NtContract.ROBOT_ENABLE_KEY).publish();

        pressDonePub.set(false);
        missionStartPub.set(false);
        missionLoadedPub.set(false);
        missionUnloadedPub.set(false);
        missionRestartPub.set(false);
        enablePub.set(false);
    }

    public String pollNewButtonRequest() {
        String current = pressButtonSub.get();
        if (current == null || current.isEmpty() || current.equals(lastButton)) {
            return null;
        }
        lastButton = current;
        return current;
    }

    public String pollNewMissionCommand() {
        String current = missionCmdSub.get();
        if (current == null || current.isEmpty() || current.equals(lastMissionCmd)) {
            return null;
        }
        lastMissionCmd = current;
        return current;
    }

    // --- pulsed events ---
    public void firePressDone() { pressDone.fire(); }
    public void fireMissionStart() { missionStart.fire(); }
    public void fireMissionLoaded() { missionLoaded.fire(); }
    public void fireMissionUnloaded() { missionUnloaded.fire(); }
    public void fireMissionRestart() { missionRestart.fire(); }

    // --- held enable ---
    public void setEnable(boolean enabled) {
        this.enabled = enabled;
        enablePub.set(enabled);
    }

    public boolean isEnabled() {
        return enabled;
    }

    /** Call every robot loop to maintain the high/low edges. */
    public void periodic() {
        pressDonePub.set(pressDone.update());
        missionStartPub.set(missionStart.update());
        missionLoadedPub.set(missionLoaded.update());
        missionUnloadedPub.set(missionUnloaded.update());
        missionRestartPub.set(missionRestart.update());
    }

    private static final class PulseState {
        private double riseFpga = Double.NEGATIVE_INFINITY;

        void fire() {
            riseFpga = Timer.getFPGATimestamp();
        }

        boolean update() {
            return (Timer.getFPGATimestamp() - riseFpga) < PULSE_HOLD_SEC;
        }
    }
}
