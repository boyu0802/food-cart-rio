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
 * Horizontal pusher on top of the small lift — 1x NEO Vortex.
 * Pushes lunches out, pulls them back in.
 *
 * Treat "extended" / "retracted" as named positions; tune in MissionSupervisor.
 */
public class PusherSubsystem extends SubsystemBase {
    private final SparkFlex motor;
    private final RelativeEncoder encoder;
    private final SparkClosedLoopController controller;

    private double targetRotations = 0.0;
    private boolean closedLoopActive = false;

    private final SysIdRoutine sysIdRoutine;

    public PusherSubsystem(int canId, double kP) {
        motor = new SparkFlex(canId, MotorType.kBrushless);
        SparkFlexConfig cfg = new SparkFlexConfig();
        cfg.idleMode(IdleMode.kBrake).smartCurrentLimit(40);
        cfg.closedLoop.pid(kP, 0.0, 0.0).outputRange(-1.0, 1.0);
        motor.configure(cfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        encoder = motor.getEncoder();
        controller = motor.getClosedLoopController();
        encoder.setPosition(0.0);

        sysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(0.5).per(Seconds),
                Volts.of(3.0),
                Seconds.of(2.0),
                state -> Logger.recordOutput("Pusher/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                volts -> motor.setVoltage(volts.in(Volts)),
                log -> log.motor("pusher")
                    .voltage(Volts.of(motor.getAppliedOutput() * motor.getBusVoltage()))
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
        motor.set(percent);
    }

    public void stop() {
        closedLoopActive = false;
        motor.set(0.0);
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
    }
}
