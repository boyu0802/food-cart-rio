package frc.robot.subsystems.lift;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

/**
 * Linear lift driven by a leader NEO Vortex + N followers (SparkFlex).
 * Today it's 1 leader + 1 inverted follower (motors on opposite sides).
 *
 * Position units are rotations of the leader motor. Soft limits (in those same
 * rotations) are enforced on the leader in every control mode — the encoder is
 * zeroed at boot, so the mechanism MUST start at its home position or the limits
 * will be off. Calibrate the rotations->meters conversion in MissionSequences.
 */
public class LiftSubsystem extends SubsystemBase {
    private final String name;
    private final SparkFlex leader;
    private final SparkFlex[] followers;
    private final RelativeEncoder encoder;
    private final SparkClosedLoopController controller;

    private double targetRotations = 0.0;
    private boolean closedLoopActive = false;

    private final SysIdRoutine sysIdRoutine;

    /**
     * @param name             logging name, e.g. "Lift"
     * @param leaderCanId      CAN ID of the leader SparkFlex
     * @param followerCanIds   CAN IDs of followers
     * @param followerInverted whether each follower runs opposite to the leader
     * @param kP               position-loop P gain (rotations -> output)
     * @param minRotations     reverse soft limit (leader rotations); ~0 at home
     * @param maxRotations     forward soft limit (leader rotations); top of travel
     */
    public LiftSubsystem(String name, int leaderCanId, int[] followerCanIds,
                         boolean[] followerInverted, double kP,
                         double minRotations, double maxRotations) {
        this.name = name;
        if (followerCanIds.length != followerInverted.length) {
            throw new IllegalArgumentException("followerCanIds and followerInverted must match length");
        }

        leader = new SparkFlex(leaderCanId, MotorType.kBrushless);
        SparkFlexConfig leaderCfg = new SparkFlexConfig();
        leaderCfg.idleMode(IdleMode.kBrake).smartCurrentLimit(60);
        leaderCfg.closedLoop.pid(kP, 0.0, 0.0).outputRange(-1.0, 1.0);
        leaderCfg.softLimit
            .forwardSoftLimit(maxRotations).forwardSoftLimitEnabled(true)
            .reverseSoftLimit(minRotations).reverseSoftLimitEnabled(true);
        leader.configure(leaderCfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        followers = new SparkFlex[followerCanIds.length];
        for (int i = 0; i < followerCanIds.length; i++) {
            followers[i] = new SparkFlex(followerCanIds[i], MotorType.kBrushless);
            SparkFlexConfig fCfg = new SparkFlexConfig();
            fCfg.idleMode(IdleMode.kBrake).smartCurrentLimit(60);
            fCfg.follow(leader, followerInverted[i]);
            followers[i].configure(fCfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        }

        encoder = leader.getEncoder();
        controller = leader.getClosedLoopController();
        encoder.setPosition(0.0);

        sysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(0.5).per(Seconds),  // ramp rate (slow on lifts to keep them safe)
                Volts.of(3.0),                // dynamic step (mild)
                Seconds.of(3.0),              // timeout
                state -> Logger.recordOutput("Lift/" + name + "/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                volts -> leader.setVoltage(volts.in(Volts)),
                log -> log.motor("lift-" + name)
                    .voltage(Volts.of(leader.getAppliedOutput() * leader.getBusVoltage()))
                    .angularPosition(Rotations.of(encoder.getPosition()))
                    .angularVelocity(RotationsPerSecond.of(encoder.getVelocity() / 60.0)),
                this));
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.dynamic(direction);
    }

    /** Drive to a target leader-rotation position. */
    public void setPosition(double rotations) {
        targetRotations = rotations;
        closedLoopActive = true;
        controller.setSetpoint(rotations, ControlType.kPosition);
    }

    /** Open-loop test / jog (percent of bus voltage). Disables position hold. */
    public void setOpenLoop(double percent) {
        closedLoopActive = false;
        leader.set(percent);
    }

    public void stop() {
        closedLoopActive = false;
        leader.set(0.0);
    }

    public double getPosition() {
        return encoder.getPosition();
    }

    public double getVelocity() {
        return encoder.getVelocity();
    }

    public boolean atPosition(double toleranceRotations) {
        return closedLoopActive && Math.abs(getPosition() - targetRotations) <= toleranceRotations;
    }

    public void zero() {
        encoder.setPosition(0.0);
    }

    @Override
    public void periodic() {
        Logger.recordOutput("Lift/" + name + "/position", getPosition());
        Logger.recordOutput("Lift/" + name + "/velocity", getVelocity());
        Logger.recordOutput("Lift/" + name + "/target", targetRotations);
        Logger.recordOutput("Lift/" + name + "/closedLoop", closedLoopActive);
        Logger.recordOutput("Lift/" + name + "/leaderCurrent", leader.getOutputCurrent());
        for (int i = 0; i < followers.length; i++) {
            Logger.recordOutput("Lift/" + name + "/follower" + i + "Current", followers[i].getOutputCurrent());
        }
    }
}
