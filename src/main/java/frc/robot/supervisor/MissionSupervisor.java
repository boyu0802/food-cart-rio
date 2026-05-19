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
 * Top-level mission coordinator. Two responsibilities:
 *
 *  1. Run the button-press state machine for the {@link ButtonPresserSubsystem}
 *     whenever the Pi publishes a string to {@code Pi/elevator/press_button}.
 *  2. Listen for higher-level Pi mission commands (load / unload / stow) on
 *     {@code Pi/mission/cmd} and schedule the matching sequence on the lift +
 *     pusher. Each sequence pulses the corresponding mission event on NT when
 *     it finishes.
 *
 * Physical sensors on the cart (start button, lunch-loaded, lunch-unloaded)
 * are also polled here and forwarded to the Pi as rising-edge events.
 */
public class MissionSupervisor extends SubsystemBase {
    public enum PressState { IDLE, LIFT_TO_HEIGHT, EXTEND_POKER, HOLD, RETRACT_POKER, LIFT_HOME, DONE }

    private static final double LIFT_TOLERANCE_ROT = 0.5;
    private static final double POKER_EXTEND_TIME_SEC = 0.25;
    private static final double HOLD_TIME_SEC = 0.4;
    private static final double POKER_RETRACT_TIME_SEC = 0.25;
    private static final double LIFT_HOME_ROT = 0.0;

    // press_button name -> presser-lift target. TUNE per elevator car.
    private final Map<String, Double> buttonHeights = new HashMap<>();

    private final ButtonPresserSubsystem presser;
    private final LiftSubsystem lift;
    private final PusherSubsystem pusher;
    private final MissionIO missionIo;

    private final DigitalInput startBtn;
    private final DigitalInput loadedSensor;
    private final DigitalInput unloadedSensor;
    private boolean lastStart = false;
    private boolean lastLoaded = false;
    private boolean lastUnloaded = false;

    private PressState pressState = PressState.IDLE;
    private double pressStateEnteredFpga = 0.0;
    private String activeButton = "";

    private Command activeMissionSequence = null;

    /**
     * @param presser     button-press arm
     * @param lift        lunch lift (2 NEO Vortex)
     * @param pusher      horizontal pusher
     * @param missionIo   NT mission channel
     * @param startDio    DIO channel for "start mission" button. -1 to skip.
     * @param loadedDio   DIO channel for lunch-present sensor. -1 to skip.
     * @param unloadedDio DIO channel for "lunch removed" sensor. -1 to skip.
     */
    public MissionSupervisor(ButtonPresserSubsystem presser,
                             LiftSubsystem lift,
                             PusherSubsystem pusher,
                             MissionIO missionIo,
                             int startDio, int loadedDio, int unloadedDio) {
        this.presser = presser;
        this.lift = lift;
        this.pusher = pusher;
        this.missionIo = missionIo;
        this.startBtn = startDio >= 0 ? new DigitalInput(startDio) : null;
        this.loadedSensor = loadedDio >= 0 ? new DigitalInput(loadedDio) : null;
        this.unloadedSensor = unloadedDio >= 0 ? new DigitalInput(unloadedDio) : null;

        buttonHeights.put("call_up", 10.0);
        buttonHeights.put("call_down", 8.0);
        buttonHeights.put("floor_1", 6.0);
        buttonHeights.put("floor_2", 7.0);
        buttonHeights.put("floor_3", 9.0);
        buttonHeights.put("floor_4", 11.0);
    }

    @Override
    public void periodic() {
        pollPhysicalSensors();
        dispatchMissionCommand();
        stepPressMachine();
        missionIo.periodic();

        Logger.recordOutput("Mission/pressState", pressState.toString());
        Logger.recordOutput("Mission/activeButton", activeButton);
        Logger.recordOutput("Mission/activeSequence",
            activeMissionSequence == null ? "" : activeMissionSequence.getName());
    }

    /** True while a Pi-issued load/unload/stow sequence is running. */
    public boolean isMissionSequenceRunning() {
        return activeMissionSequence != null && activeMissionSequence.isScheduled();
    }

    public PressState getPressState() {
        return pressState;
    }

    // ---- internals ----

    private void pollPhysicalSensors() {
        if (startBtn != null) {
            boolean cur = !startBtn.get();
            if (cur && !lastStart) missionIo.fireMissionStart();
            lastStart = cur;
        }
        if (loadedSensor != null) {
            boolean cur = !loadedSensor.get();
            if (cur && !lastLoaded) missionIo.fireMissionLoaded();
            lastLoaded = cur;
        }
        if (unloadedSensor != null) {
            boolean cur = !unloadedSensor.get();
            if (cur && !lastUnloaded) missionIo.fireMissionUnloaded();
            lastUnloaded = cur;
        }
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
