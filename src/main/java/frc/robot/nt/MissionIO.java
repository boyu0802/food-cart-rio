package frc.robot.nt;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.wpilibj.Timer;

/**
 * Mission-channel half of the nt_bridge contract:
 *   in  : Pi/elevator/press_button (String)        — name of the button the Pi asked us to press
 *   out : Robot/elevator/press_done (bool, pulsed) — fires once when the press finishes
 *   out : Robot/mission/{start_pressed, loaded, unloaded} (bool, pulsed)
 *
 * The Pi side reads these booleans edge-triggered (False -> True). So to send an "event":
 *   set the topic true, hold ~one bridge poll period (defaults to 1/20 s on the Pi),
 *   then set it back to false. {@link #pulse} encapsulates that.
 */
public class MissionIO {
    /** Hold-true duration for an edge pulse. 100 ms is well above the Pi's 50 ms poll. */
    public static final double PULSE_HOLD_SEC = 0.1;

    private final StringSubscriber pressButtonSub;
    private final StringSubscriber missionCmdSub;

    private final BooleanPublisher pressDonePub;
    private final BooleanPublisher missionStartPub;
    private final BooleanPublisher missionLoadedPub;
    private final BooleanPublisher missionUnloadedPub;

    private final PulseState pressDone = new PulseState();
    private final PulseState missionStart = new PulseState();
    private final PulseState missionLoaded = new PulseState();
    private final PulseState missionUnloaded = new PulseState();

    private String lastButton = "";
    private String lastMissionCmd = "";

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

        pressDonePub.set(false);
        missionStartPub.set(false);
        missionLoadedPub.set(false);
        missionUnloadedPub.set(false);
    }

    /** Returns a new button-press request if the Pi just published one, else null. */
    public String pollNewButtonRequest() {
        String current = pressButtonSub.get();
        if (current == null || current.isEmpty() || current.equals(lastButton)) {
            return null;
        }
        lastButton = current;
        return current;
    }

    /** Returns a new mission command from the Pi ("load", "unload", "stow", ...) or null. */
    public String pollNewMissionCommand() {
        String current = missionCmdSub.get();
        if (current == null || current.isEmpty() || current.equals(lastMissionCmd)) {
            return null;
        }
        lastMissionCmd = current;
        return current;
    }

    public void firePressDone() { pressDone.fire(); }
    public void fireMissionStart() { missionStart.fire(); }
    public void fireMissionLoaded() { missionLoaded.fire(); }
    public void fireMissionUnloaded() { missionUnloaded.fire(); }

    /** Call every robot loop to maintain the high/low edges. */
    public void periodic() {
        pressDonePub.set(pressDone.update());
        missionStartPub.set(missionStart.update());
        missionLoadedPub.set(missionLoaded.update());
        missionUnloadedPub.set(missionUnloaded.update());
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
