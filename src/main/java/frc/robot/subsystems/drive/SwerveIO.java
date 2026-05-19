package frc.robot.subsystems.drive;

import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.util.SubsystemDataProcessor.IoRefresher;

public interface SwerveIO extends IoRefresher {

    @AutoLog
    class SwerveIOInputs {
        public Pose2d estimatedRobotPose = new Pose2d();
        public SwerveModuleState[] moduleStates = new SwerveModuleState[0];
        public ChassisSpeeds robotRelativeChassisSpeeds = new ChassisSpeeds();
        public ChassisSpeeds fieldRelativeChassisSpeeds = new ChassisSpeeds();
        public SwerveModuleState[] targetStates = new SwerveModuleState[0];
        public SwerveModulePosition[] modulePositions = new SwerveModulePosition[0];
        public Rotation2d gyroAngle = new Rotation2d();
        public double yawRateRadPerSec = 0.0;
        public double accelXMetersPerSecSq = 0.0;
        public double accelYMetersPerSecSq = 0.0;
        public double timeStamp = 0.0;
    }

    @AutoLog
    class ModuleIOInputs {
        public double driveId;
        public double drivePosition;
        public double driveVelocity;
        public double driveAppliedVolts;
        public double steerId;
        public double steerAbsolutePositionRotations;
        public double steerPositionRotations;
        public double steerVelocity;
        public double steerAppliedVolts;
    }

    Pose2d getPose();

    void updateSwerveInputs(SwerveIOInputs inputs);

    void updateModuleInputs(ModuleIOInputs... inputs);

    Command applyRequest(Supplier<SwerveRequest> requestSupplier, Subsystem requiredSubsystem);

    void driveRequest(SwerveRequest requestSupplier);

    void runCharacterization(SwerveRequest request);

    Command setFieldCentric();

    Rotation2d getGyro();

    default void periodic() {}

    void addVisionMeasurement(Pose2d visionRobotPose, double timeStamp);

    @Override
    void refreshData();
}
