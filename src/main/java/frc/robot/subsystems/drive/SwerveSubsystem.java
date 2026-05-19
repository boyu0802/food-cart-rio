package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.SysIdSwerveRotation;
import com.ctre.phoenix6.swerve.SwerveRequest.SysIdSwerveSteerGains;
import com.ctre.phoenix6.swerve.SwerveRequest.SysIdSwerveTranslation;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.util.SubsystemDataProcessor;

public class SwerveSubsystem extends SubsystemBase {
    private final SwerveIOInputsAutoLogged inputs = new SwerveIOInputsAutoLogged();
    private final ModuleIOInputsAutoLogged flInputs = new ModuleIOInputsAutoLogged();
    private final ModuleIOInputsAutoLogged blInputs = new ModuleIOInputsAutoLogged();
    private final ModuleIOInputsAutoLogged brInputs = new ModuleIOInputsAutoLogged();
    private final ModuleIOInputsAutoLogged frInputs = new ModuleIOInputsAutoLogged();

    private final SwerveIO io;
    private final Object lock = new Object();

    private final SwerveRequest.SysIdSwerveSteerGains kSteerSysid = new SysIdSwerveSteerGains();
    private final SwerveRequest.SysIdSwerveTranslation kDriveSysid = new SysIdSwerveTranslation();
    @SuppressWarnings("unused")
    private final SwerveRequest.SysIdSwerveRotation kRotationSysid = new SysIdSwerveRotation();

    private final SysIdRoutine m_sysIdRoutineTranslation;
    @SuppressWarnings("unused")
    private final SysIdRoutine m_sysIdRoutineSteer;
    private final SysIdRoutine m_sysIdRoutineToApply;

    public SwerveSubsystem(SwerveIO io) {
        this.io = io;

        m_sysIdRoutineTranslation = new SysIdRoutine(
            new SysIdRoutine.Config(
                null, Volts.of(4), Seconds.of(5.0),
                state -> Logger.recordOutput("Drive/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                output -> io.runCharacterization(kDriveSysid.withVolts(output)), null, this));

        m_sysIdRoutineSteer = new SysIdRoutine(
            new SysIdRoutine.Config(
                null, Volts.of(7), null,
                state -> Logger.recordOutput("Drive/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                volts -> io.runCharacterization(kSteerSysid.withVolts(volts)), null, this));

        m_sysIdRoutineToApply = m_sysIdRoutineTranslation;

        SubsystemDataProcessor.createSubsystemDataProcessor(io, () -> {
            synchronized (lock) {
                io.updateModuleInputs(flInputs, blInputs, brInputs, frInputs);
            }
        });
    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return io.applyRequest(requestSupplier, this);
    }

    public void driveRequest(SwerveRequest request) {
        io.driveRequest(request);
    }

    public Command setFieldCentric() {
        return io.setFieldCentric();
    }

    @Override
    public void periodic() {
        io.updateSwerveInputs(inputs);
        synchronized (lock) {
            Logger.processInputs("Drive/Swerve", inputs);
            Logger.processInputs("Drive/Swerve/Modules/FL", flInputs);
            Logger.processInputs("Drive/Swerve/Modules/BL", blInputs);
            Logger.processInputs("Drive/Swerve/Modules/BR", brInputs);
            Logger.processInputs("Drive/Swerve/Modules/FR", frInputs);
        }
    }

    public Pose2d getPose() {
        return inputs.estimatedRobotPose;
    }

    /** Body-frame chassis speeds — what nt_bridge wants for Robot/odom/vx,vy,omega. */
    public ChassisSpeeds getRobotRelativeSpeeds() {
        return inputs.robotRelativeChassisSpeeds;
    }

    public double getYawRadians() {
        return inputs.gyroAngle.getRadians();
    }

    public double getYawRateRadPerSec() {
        return inputs.yawRateRadPerSec;
    }

    public double getAccelXMetersPerSecSq() {
        return inputs.accelXMetersPerSecSq;
    }

    public double getAccelYMetersPerSecSq() {
        return inputs.accelYMetersPerSecSq;
    }

    public void addVisionMeasurement(Pose2d visionMeasurement, double timestampSeconds) {
        io.addVisionMeasurement(visionMeasurement, timestampSeconds);
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.dynamic(direction);
    }
}
