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
 * Horizontal pusher on top of the lift — 2x NEO Vortex (leader + follower).
 * Pushes lunches out, pulls them back in. The follower mirrors the leader and is
 * inverted (motors face opposite directions across the mechanism).
 *
 * Position units are leader rotations. Soft limits (same units) are enforced on
 * the leader in every control mode; the encoder zeros at boot, so the pusher MUST
 * start fully retracted (or wherever you call "home") for the limits to be right.
 * Treat "extended" / "retracted" as named positions; tune in MissionSequences.
 */
public class PusherSubsystem extends SubsystemBase {
    private final SparkFlex leader;
    private final SparkFlex follower;
    private final RelativeEncoder encoder;
    private final SparkClosedLoopController controller;

    private double targetRotations = 0.0;
    private boolean closedLoopActive = false;

    private final SysIdRoutine sysIdRoutine;

    /**
     * @param leaderCanId      CAN ID of the leader SparkFlex
     * @param followerCanId    CAN ID of the follower SparkFlex
     * @param followerInverted whether the follower runs opposite to the leader
     * @param kP               position-loop P gain
     * @param minRotations     reverse soft limit (leader rotations); ~0 retracted
     * @param maxRotations     forward soft limit (leader rotations); fully extended
     */
    public PusherSubsystem(int leaderCanId, int followerCanId, boolean followerInverted, double kP,
                           double minRotations, double maxRotations) {
        leader = new SparkFlex(leaderCanId, MotorType.kBrushless);
        SparkFlexConfig leaderCfg = new SparkFlexConfig();
        leaderCfg.idleMode(IdleMode.kBrake).smartCurrentLimit(40);
        leaderCfg.closedLoop.pid(kP, 0.0, 0.0).outputRange(-1.0, 1.0);
        leaderCfg.softLimit
            .forwardSoftLimit(maxRotations).forwardSoftLimitEnabled(true)
            .reverseSoftLimit(minRotations).reverseSoftLimitEnabled(true);
        leader.configure(leaderCfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        follower = new SparkFlex(followerCanId, MotorType.kBrushless);
        SparkFlexConfig followerCfg = new SparkFlexConfig();
        followerCfg.idleMode(IdleMode.kBrake).smartCurrentLimit(40);
        followerCfg.follow(leader, followerInverted);
        follower.configure(followerCfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        encoder = leader.getEncoder();
        controller = leader.getClosedLoopController();
        encoder.setPosition(0.0);

        sysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(0.5).per(Seconds),
                Volts.of(3.0),
                Seconds.of(2.0),
                state -> Logger.recordOutput("Pusher/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                volts -> leader.setVoltage(volts.in(Volts)),
                log -> log.motor("pusher")
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

    public void setPosition(double rotations) {
        targetRotations = rotations;
        closedLoopActive = true;
        controller.setSetpoint(rotations, ControlType.kPosition);
    }

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

    public boolean atPosition(double toleranceRotations) {
        return closedLoopActive && Math.abs(getPosition() - targetRotations) <= toleranceRotations;
    }

    public void zero() {
        encoder.setPosition(0.0);
    }

    @Override
    public void periodic() {
        Logger.recordOutput("Pusher/position", getPosition());
        Logger.recordOutput("Pusher/target", targetRotations);
        Logger.recordOutput("Pusher/closedLoop", closedLoopActive);
        Logger.recordOutput("Pusher/leaderCurrent", leader.getOutputCurrent());
        Logger.recordOutput("Pusher/followerCurrent", follower.getOutputCurrent());
    }
}
