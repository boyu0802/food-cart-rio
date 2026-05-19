package frc.robot.subsystems.lift;

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

import edu.wpi.first.wpilibj2.command.SubsystemBase;

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

    public PusherSubsystem(int canId, double kP) {
        motor = new SparkFlex(canId, MotorType.kBrushless);
        SparkFlexConfig cfg = new SparkFlexConfig();
        cfg.idleMode(IdleMode.kBrake).smartCurrentLimit(40);
        cfg.closedLoop.pid(kP, 0.0, 0.0).outputRange(-1.0, 1.0);
        motor.configure(cfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        encoder = motor.getEncoder();
        controller = motor.getClosedLoopController();
        encoder.setPosition(0.0);
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
