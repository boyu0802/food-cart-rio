package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.nt.MissionIO;
import frc.robot.subsystems.lift.LiftSubsystem;
import frc.robot.subsystems.lift.PusherSubsystem;

/**
 * Command factories for the Pi-driven mission sequences: load a lunch, unload a
 * lunch, stow to travel pose. Each sequence pulses the matching event on
 * {@link MissionIO} when it finishes so the Pi can advance its nav plan.
 *
 * Positions are in leader-motor rotations — TUNE the constants here once you
 * have real heights for your robot.
 */
public final class MissionSequences {
    private MissionSequences() {}

    // Lift positions (TUNE)
    public static final double LIFT_TRAVEL_ROT = 4.0;   // safe height during driving
    public static final double LIFT_LOAD_ROT = 0.5;     // low — at lunch pickup
    public static final double LIFT_DELIVERY_ROT = 6.0; // height of the destination shelf

    // Pusher positions (TUNE)
    public static final double PUSHER_RETRACTED_ROT = 0.0;
    public static final double PUSHER_EXTENDED_ROT = 5.0;

    // Tolerances
    public static final double LIFT_TOLERANCE_ROT = 0.5;
    public static final double PUSHER_TOLERANCE_ROT = 0.3;

    // How long to wait for a sensor to confirm lunch presence/absence before
    // giving up and assuming the worst.
    public static final double SENSOR_TIMEOUT_SEC = 5.0;

    /**
     * Pick up a lunch.
     *   retract pusher (clear) -> lift down to load -> wait for lunch present
     *   -> extend pusher to pull lunch in -> lift up to travel -> fire `loaded`.
     */
    public static Command load(LiftSubsystem lift, PusherSubsystem pusher, MissionIO io,
                               BooleanSupplier lunchPresent) {
        return Commands.sequence(
            Commands.runOnce(() -> pusher.setPosition(PUSHER_RETRACTED_ROT), pusher),
            Commands.waitUntil(() -> pusher.atPosition(PUSHER_TOLERANCE_ROT)),

            Commands.runOnce(() -> lift.setPosition(LIFT_LOAD_ROT), lift),
            Commands.waitUntil(() -> lift.atPosition(LIFT_TOLERANCE_ROT)),

            Commands.waitUntil(lunchPresent).withTimeout(SENSOR_TIMEOUT_SEC),

            Commands.runOnce(() -> pusher.setPosition(PUSHER_EXTENDED_ROT), pusher),
            Commands.waitUntil(() -> pusher.atPosition(PUSHER_TOLERANCE_ROT)),

            Commands.runOnce(() -> lift.setPosition(LIFT_TRAVEL_ROT), lift),
            Commands.waitUntil(() -> lift.atPosition(LIFT_TOLERANCE_ROT)),

            Commands.runOnce(io::fireMissionLoaded)
        ).withName("MissionLoad");
    }

    /**
     * Drop off a lunch.
     *   lift down to delivery -> extend pusher -> wait for lunch gone
     *   -> retract pusher -> lift up to travel -> fire `unloaded`.
     */
    public static Command unload(LiftSubsystem lift, PusherSubsystem pusher, MissionIO io,
                                 BooleanSupplier lunchPresent) {
        return Commands.sequence(
            Commands.runOnce(() -> lift.setPosition(LIFT_DELIVERY_ROT), lift),
            Commands.waitUntil(() -> lift.atPosition(LIFT_TOLERANCE_ROT)),

            Commands.runOnce(() -> pusher.setPosition(PUSHER_EXTENDED_ROT), pusher),
            Commands.waitUntil(() -> pusher.atPosition(PUSHER_TOLERANCE_ROT)),

            Commands.waitUntil(() -> !lunchPresent.getAsBoolean()).withTimeout(SENSOR_TIMEOUT_SEC),

            Commands.runOnce(() -> pusher.setPosition(PUSHER_RETRACTED_ROT), pusher),
            Commands.waitUntil(() -> pusher.atPosition(PUSHER_TOLERANCE_ROT)),

            Commands.runOnce(() -> lift.setPosition(LIFT_TRAVEL_ROT), lift),
            Commands.waitUntil(() -> lift.atPosition(LIFT_TOLERANCE_ROT)),

            Commands.runOnce(io::fireMissionUnloaded)
        ).withName("MissionUnload");
    }

    /** Return lift + pusher to safe driving configuration. */
    public static Command stow(LiftSubsystem lift, PusherSubsystem pusher) {
        return Commands.sequence(
            Commands.runOnce(() -> pusher.setPosition(PUSHER_RETRACTED_ROT), pusher),
            Commands.waitUntil(() -> pusher.atPosition(PUSHER_TOLERANCE_ROT)),

            Commands.runOnce(() -> lift.setPosition(LIFT_TRAVEL_ROT), lift),
            Commands.waitUntil(() -> lift.atPosition(LIFT_TOLERANCE_ROT))
        ).withName("MissionStow");
    }
}
