package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.nt.MissionIO;
import frc.robot.subsystems.lift.LiftSubsystem;
import frc.robot.subsystems.lift.LunchLockSubsystem;
import frc.robot.subsystems.lift.PusherSubsystem;

/**
 * Command factories for the Pi-driven mission sequences: load a lunch, unload a
 * lunch, stow to travel pose. Each sequence pulses the matching event on
 * {@link MissionIO} when it finishes so the Pi can advance its nav plan.
 *
 * Pusher convention: RETRACTED = pulled into the robot (home), EXTENDED = reached
 * out toward the shelf. So "pull the lunch in" = retract, "push it out" = extend.
 *
 * Positions are in leader-motor rotations — TUNE the constants here once you have
 * real heights for your robot. The load/unload step ORDER is a scaffold from the
 * description "open robot-lock -> go to position -> grab with transfer-lock ->
 * pull back -> lock robot-lock"; adjust as you test.
 */
public final class MissionSequences {
    private MissionSequences() {}

    // Lift positions (TUNE)
    public static final double LIFT_TRAVEL_ROT = 4.0;
    public static final double LIFT_LOAD_ROT = 0.5;
    public static final double LIFT_DELIVERY_ROT = 6.0;

    // Pusher positions (TUNE)
    public static final double PUSHER_RETRACTED_ROT = 0.0;  // pulled into robot
    public static final double PUSHER_EXTENDED_ROT = 5.0;   // reached out

    // Tolerances
    public static final double LIFT_TOLERANCE_ROT = 0.5;
    public static final double PUSHER_TOLERANCE_ROT = 0.3;

    // Sensor wait fallback, and pneumatic actuation dwell.
    public static final double SENSOR_TIMEOUT_SEC = 5.0;
    public static final double LOCK_DWELL_SEC = 0.3;

    /**
     * Pick up a lunch.
     *   unlock robot-lock + open transfer-lock -> reach pusher out -> lift to load
     *   -> wait for lunch -> grip with transfer-lock -> pull pusher in
     *   -> lock robot-lock -> lift to travel -> fire `loaded`.
     */
    public static Command load(LiftSubsystem lift, PusherSubsystem pusher,
                               LunchLockSubsystem locks, MissionIO io,
                               BooleanSupplier lunchPresent) {
        return Commands.sequence(
            Commands.runOnce(locks::unlockRobot),
            Commands.runOnce(locks::unlockTransfer),
            Commands.waitSeconds(LOCK_DWELL_SEC),

            Commands.runOnce(() -> pusher.setPosition(PUSHER_EXTENDED_ROT), pusher),
            Commands.waitUntil(() -> pusher.atPosition(PUSHER_TOLERANCE_ROT)),

            Commands.runOnce(() -> lift.setPosition(LIFT_LOAD_ROT), lift),
            Commands.waitUntil(() -> lift.atPosition(LIFT_TOLERANCE_ROT)),

            Commands.waitUntil(lunchPresent).withTimeout(SENSOR_TIMEOUT_SEC),

            Commands.runOnce(locks::lockTransfer),
            Commands.waitSeconds(LOCK_DWELL_SEC),

            Commands.runOnce(() -> pusher.setPosition(PUSHER_RETRACTED_ROT), pusher),
            Commands.waitUntil(() -> pusher.atPosition(PUSHER_TOLERANCE_ROT)),

            Commands.runOnce(locks::lockRobot),
            Commands.waitSeconds(LOCK_DWELL_SEC),

            Commands.runOnce(() -> lift.setPosition(LIFT_TRAVEL_ROT), lift),
            Commands.waitUntil(() -> lift.atPosition(LIFT_TOLERANCE_ROT)),

            Commands.runOnce(io::fireMissionLoaded)
        ).withName("MissionLoad");
    }

    /**
     * Drop off a lunch.
     *   lift to delivery -> unlock robot-lock -> push pusher out -> release
     *   transfer-lock -> wait for lunch gone -> pull pusher in -> lift to travel
     *   -> fire `unloaded`.
     */
    public static Command unload(LiftSubsystem lift, PusherSubsystem pusher,
                                 LunchLockSubsystem locks, MissionIO io,
                                 BooleanSupplier lunchPresent) {
        return Commands.sequence(
            Commands.runOnce(() -> lift.setPosition(LIFT_DELIVERY_ROT), lift),
            Commands.waitUntil(() -> lift.atPosition(LIFT_TOLERANCE_ROT)),

            Commands.runOnce(locks::unlockRobot),
            Commands.waitSeconds(LOCK_DWELL_SEC),

            Commands.runOnce(() -> pusher.setPosition(PUSHER_EXTENDED_ROT), pusher),
            Commands.waitUntil(() -> pusher.atPosition(PUSHER_TOLERANCE_ROT)),

            Commands.runOnce(locks::unlockTransfer),
            Commands.waitSeconds(LOCK_DWELL_SEC),

            Commands.waitUntil(() -> !lunchPresent.getAsBoolean()).withTimeout(SENSOR_TIMEOUT_SEC),

            Commands.runOnce(() -> pusher.setPosition(PUSHER_RETRACTED_ROT), pusher),
            Commands.waitUntil(() -> pusher.atPosition(PUSHER_TOLERANCE_ROT)),

            Commands.runOnce(() -> lift.setPosition(LIFT_TRAVEL_ROT), lift),
            Commands.waitUntil(() -> lift.atPosition(LIFT_TOLERANCE_ROT)),

            Commands.runOnce(io::fireMissionUnloaded)
        ).withName("MissionUnload");
    }

    /** Return lift + pusher to safe driving configuration; keep the lunch locked in. */
    public static Command stow(LiftSubsystem lift, PusherSubsystem pusher, LunchLockSubsystem locks) {
        return Commands.sequence(
            Commands.runOnce(() -> pusher.setPosition(PUSHER_RETRACTED_ROT), pusher),
            Commands.waitUntil(() -> pusher.atPosition(PUSHER_TOLERANCE_ROT)),

            Commands.runOnce(locks::lockRobot),

            Commands.runOnce(() -> lift.setPosition(LIFT_TRAVEL_ROT), lift),
            Commands.waitUntil(() -> lift.atPosition(LIFT_TOLERANCE_ROT))
        ).withName("MissionStow");
    }
}
