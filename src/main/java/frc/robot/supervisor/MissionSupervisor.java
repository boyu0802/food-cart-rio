package frc.robot.supervisor;

import java.util.HashMap;
import java.util.Map;
import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.MissionSequences;
import frc.robot.nt.MissionIO;
import frc.robot.subsystems.lift.LiftSubsystem;
import frc.robot.subsystems.lift.PusherSubsystem;
import frc.robot.subsystems.presser.ButtonPresserSubsystem;

/**
 * Top-level mission coordinator with dead-man enable.
 *
 *  - Listens for {@code Pi/elevator/press_button} → runs the button-press state
 *    machine on the {@link ButtonPresserSubsystem}, then pulses {@code press_done}.
 *  - Listens for {@code Pi/mission/cmd} ("load" / "unload" / "stow") → schedules
 *    the matching {@link MissionSequences} command, which pulses
 *    {@code loaded}/{@code unloaded} on completion.
 *  - Watches the {@code startBtn} DIO and pulses {@code mission/start_pressed}
 *    on its rising edge. (The {@code loadedSensor} / {@code unloadedSensor} are
 *    still read — only used as the {@link BooleanSupplier} for waitUntil inside
 *    sequences — they DO NOT pulse loaded/unloaded any more, since the sequence
 *    fires those at completion and we don't want a double-edge.)
 *  - Dead-man: while {@code missionIo.isEnabled() == false}, the press machine
 *    is frozen and the active mission sequence is canceled. Motors stop. No
 *    completion events fire.
 *  - {@link #restart()} aborts everything and returns to IDLE.
 */
public class MissionSupervisor extends SubsystemBase {
    public enum PressState { IDLE, LIFT_TO_HEIGHT, EXTEND_POKER, HOLD, RETRACT_POKER, LIFT_HOME, DONE }

    private static final double LIFT_TOLERANCE_ROT = 0.5;
    private static final double POKER_EXTEND_TIME_SEC = 0.25;
    private static final double HOLD_TIME_SEC = 0.4;
    private static final double POKER_RETRACT_TIME_SEC = 0.25;
    private static final double LIFT_HOME_ROT = 0.0;

    private final Map<String, Double> buttonHeights = new HashMap<>();

    private final ButtonPresserSubsystem presser;
    private final LiftSubsystem lift;
    private final PusherSubsystem pusher;
    private final MissionIO missionIo;

    private final DigitalInput startBtn;
    private final DigitalInput loadedSensor;
    private boolean lastStart = false;

    private PressState pressState = PressState.IDLE;
    private double pressStateEnteredFpga = 0.0;
    private String activeButton = "";

    private Command activeMissionSequence = null;
    private boolean wasEnabled = false;

    public MissionSupervisor(ButtonPresserSubsystem presser,
                             LiftSubsystem lift,
                             PusherSubsystem pusher,
                             MissionIO missionIo,
                             int startDio, int loadedDio) {
        this.presser = presser;
        this.lift = lift;
        this.pusher = pusher;
        this.missionIo = missionIo;
        this.startBtn = startDio >= 0 ? new DigitalInput(startDio) : null;
        this.loadedSensor = loadedDio >= 0 ? new DigitalInput(loadedDio) : null;

        buttonHeights.put("call_up", 10.0);
        buttonHeights.put("call_down", 8.0);
        buttonHeights.put("floor_1", 6.0);
        buttonHeights.put("floor_2", 7.0);
        buttonHeights.put("floor_3", 9.0);
        buttonHeights.put("floor_4", 11.0);
    }

    @Override
    public void periodic() {
        boolean enabled = missionIo.isEnabled();

        if (wasEnabled && !enabled) {
            freezeOnDisable();
        }
        wasEnabled = enabled;

        pollStartButton();              // start_pressed always reports (operator may want it any time)

        if (enabled) {
            dispatchMissionCommand();
            stepPressMachine();
        }

        missionIo.periodic();           // maintain edge pulses for everything we fired

        Logger.recordOutput("Mission/enabled", enabled);
        Logger.recordOutput("Mission/pressState", pressState.toString());
        Logger.recordOutput("Mission/activeButton", activeButton);
        Logger.recordOutput("Mission/activeSequence",
            activeMissionSequence == null ? "" : activeMissionSequence.getName());
    }

    /** Abort everything. Cancel current sequence, return press machine to IDLE, stop motors. */
    public void restart() {
        if (activeMissionSequence != null && activeMissionSequence.isScheduled()) {
            activeMissionSequence.cancel();
        }
        activeMissionSequence = null;
        pressState = PressState.IDLE;
        activeButton = "";

        presser.retractPoker();
        presser.stopLift();
        lift.stop();
        pusher.stop();

        missionIo.clearButtonDedupe();
        missionIo.clearMissionCmdDedupe();
    }

    public boolean isMissionSequenceRunning() {
        return activeMissionSequence != null && activeMissionSequence.isScheduled();
    }

    public PressState getPressState() {
        return pressState;
    }

    // ---- internals ----

    private void freezeOnDisable() {
        if (activeMissionSequence != null && activeMissionSequence.isScheduled()) {
            activeMissionSequence.cancel();
        }
        presser.retractPoker();
        presser.stopLift();
        lift.stop();
        pusher.stop();
        pressState = PressState.IDLE;
        activeButton = "";
        // Clear the de-dupe latches so that on resume the Pi can re-issue the
        // same press_button / mission/cmd string and we treat it as fresh.
        missionIo.clearButtonDedupe();
        missionIo.clearMissionCmdDedupe();
    }

    private void pollStartButton() {
        if (startBtn == null) return;
        boolean cur = !startBtn.get();
        if (cur && !lastStart) missionIo.fireMissionStart();
        lastStart = cur;
    }

    private BooleanSupplier lunchPresent() {
        return () -> loadedSensor != null && !loadedSensor.get();
    }

    private void dispatchMissionCommand() {
        String cmd = missionIo.pollNewMissionCommand();
        if (cmd == null) return;

        Command next;
        switch (cmd) {
            case "load":
                next = MissionSequences.load(lift, pusher, missionIo, lunchPresent());
                break;
            case "unload":
                next = MissionSequences.unload(lift, pusher, missionIo, lunchPresent());
                break;
            case "stow":
                next = MissionSequences.stow(lift, pusher);
                break;
            default:
                Logger.recordOutput("Mission/unknownCommand", cmd);
                return;
        }

        if (activeMissionSequence != null && activeMissionSequence.isScheduled()) {
            activeMissionSequence.cancel();
        }
        activeMissionSequence = next;
        CommandScheduler.getInstance().schedule(next);
    }

    private void stepPressMachine() {
        double now = Timer.getFPGATimestamp();

        switch (pressState) {
            case IDLE: {
                String request = missionIo.pollNewButtonRequest();
                if (request != null && buttonHeights.containsKey(request)) {
                    activeButton = request;
                    presser.retractPoker();
                    presser.setLiftPosition(buttonHeights.get(request));
                    enter(PressState.LIFT_TO_HEIGHT, now);
                } else if (request != null) {
                    Logger.recordOutput("Mission/unknownButton", request);
                }
                break;
            }
            case LIFT_TO_HEIGHT:
                if (presser.liftAtPosition(LIFT_TOLERANCE_ROT)) {
                    presser.extendPoker();
                    enter(PressState.EXTEND_POKER, now);
                }
                break;
            case EXTEND_POKER:
                if (now - pressStateEnteredFpga >= POKER_EXTEND_TIME_SEC) {
                    enter(PressState.HOLD, now);
                }
                break;
            case HOLD:
                if (now - pressStateEnteredFpga >= HOLD_TIME_SEC) {
                    presser.retractPoker();
                    enter(PressState.RETRACT_POKER, now);
                }
                break;
            case RETRACT_POKER:
                if (now - pressStateEnteredFpga >= POKER_RETRACT_TIME_SEC) {
                    presser.setLiftPosition(LIFT_HOME_ROT);
                    enter(PressState.LIFT_HOME, now);
                }
                break;
            case LIFT_HOME:
                if (presser.liftAtPosition(LIFT_TOLERANCE_ROT)) {
                    enter(PressState.DONE, now);
                }
                break;
            case DONE:
                missionIo.firePressDone();
                activeButton = "";
                enter(PressState.IDLE, now);
                break;
        }
    }

    private void enter(PressState next, double now) {
        pressState = next;
        pressStateEnteredFpga = now;
    }
}
