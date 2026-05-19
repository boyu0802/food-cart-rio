package frc.robot.subsystems.presser;

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
import edu.wpi.first.wpilibj2.command.SubsystemBase;

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
