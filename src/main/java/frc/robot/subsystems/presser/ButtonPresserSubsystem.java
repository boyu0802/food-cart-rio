package frc.robot.subsystems.presser;

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

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

/**
 * The button-press arm: a single NEO Vortex driving a vertical lift, with a
 * single-acting pneumatic poker on top to physically press elevator buttons.
 *
 * Press sequence is owned by {@link frc.robot.supervisor.MissionSupervisor}.
 * This subsystem just exposes height + poker primitives.
 */
public class ButtonPresserSubsystem extends SubsystemBase {
    private final SparkFlex motor;
    private final RelativeEncoder encoder;
    private final SparkClosedLoopController controller;
    private final Solenoid poker;

    private double targetRotations = 0.0;
    private boolean closedLoopActive = false;
    private boolean pokerExtended = false;

    private final SysIdRoutine sysIdRoutine;

    /**
     * @param canId         CAN ID of the lift NEO Vortex (SparkFlex)
     * @param kP            position-loop P gain
     * @param phModuleId    CAN ID of the REV PH (1 by default)
     * @param solenoidChan  PH solenoid channel powering the poker
     */
    public ButtonPresserSubsystem(int canId, double kP, int phModuleId, int solenoidChan) {
        motor = new SparkFlex(canId, MotorType.kBrushless);
        SparkFlexConfig cfg = new SparkFlexConfig();
        cfg.idleMode(IdleMode.kBrake).smartCurrentLimit(40);
        cfg.closedLoop.pid(kP, 0.0, 0.0).outputRange(-1.0, 1.0);
        motor.configure(cfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        encoder = motor.getEncoder();
        controller = motor.getClosedLoopController();
        encoder.setPosition(0.0);

        poker = new Solenoid(phModuleId, PneumaticsModuleType.REVPH, solenoidChan);
        poker.set(false);

        sysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(0.5).per(Seconds),
                Volts.of(3.0),
                Seconds.of(3.0),
                state -> Logger.recordOutput("ButtonPresser/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                volts -> motor.setVoltage(volts.in(Volts)),
                log -> log.motor("presser-lift")
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

    public void setLiftPosition(double rotations) {
        targetRotations = rotations;
        closedLoopActive = true;
        controller.setSetpoint(rotations, ControlType.kPosition);
    }

    public void setOpenLoop(double percent) {
        closedLoopActive = false;
        motor.set(percent);
    }

    public void stopLift() {
        closedLoopActive = false;
        motor.set(0.0);
    }

    public double getLiftPosition() {
        return encoder.getPosition();
    }

    public boolean liftAtPosition(double toleranceRotations) {
        return closedLoopActive && Math.abs(getLiftPosition() - targetRotations) <= toleranceRotations;
    }

    public void zeroLift() {
        encoder.setPosition(0.0);
    }

    public void extendPoker() {
        pokerExtended = true;
        poker.set(true);
    }

    public void retractPoker() {
        pokerExtended = false;
        poker.set(false);
    }

    public boolean isPokerExtended() {
        return pokerExtended;
    }

    @Override
    public void periodic() {
        Logger.recordOutput("ButtonPresser/liftPosition", getLiftPosition());
        Logger.recordOutput("ButtonPresser/liftTarget", targetRotations);
        Logger.recordOutput("ButtonPresser/liftClosedLoop", closedLoopActive);
        Logger.recordOutput("ButtonPresser/pokerExtended", pokerExtended);
    }
}
