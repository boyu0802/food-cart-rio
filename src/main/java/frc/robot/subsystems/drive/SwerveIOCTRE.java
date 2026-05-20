package frc.robot.subsystems.drive;

import java.util.HashMap;
import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;

import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class SwerveIOCTRE extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder> implements SwerveIO {
    private final HashMap<String, BaseStatusSignal> frontLeftSignals = new HashMap<>();
    private final HashMap<String, BaseStatusSignal> backLeftSignals = new HashMap<>();
    private final HashMap<String, BaseStatusSignal> backRightSignals = new HashMap<>();
    private final HashMap<String, BaseStatusSignal> frontRightSignals = new HashMap<>();
    private final HashMap<Integer, HashMap<String, BaseStatusSignal>> swerveModuleSignals = new HashMap<>();

    private final StatusSignal<AngularVelocity> pigeonYawRate;
    private final StatusSignal<LinearAcceleration> pigeonAccelX;
    private final StatusSignal<LinearAcceleration> pigeonAccelY;
    private final StatusSignal<LinearAcceleration> pigeonAccelZ;


    @SafeVarargs
    public SwerveIOCTRE(SwerveDrivetrainConstants constants, SwerveModuleConstants<?, ?, ?>... moduleConstants) {
        super(TalonFX::new, TalonFX::new, CANcoder::new, constants, moduleConstants);

        swerveModuleSignals.put(0, frontLeftSignals);
        swerveModuleSignals.put(1, backLeftSignals);
        swerveModuleSignals.put(2, backRightSignals);
        swerveModuleSignals.put(3, frontRightSignals);

        for (int i = 0; i < 4; i++) {
            var driveMotor = this.getModule(i).getDriveMotor();
            var angleMotor = this.getModule(i).getSteerMotor();
            var canCoder = this.getModule(i).getEncoder();
            var moduleMap = swerveModuleSignals.get(i);

            moduleMap.put("drivePosition", driveMotor.getPosition());
            moduleMap.put("driveVelocity", driveMotor.getVelocity());
            moduleMap.put("driveCurrent", driveMotor.getSupplyCurrent());
            moduleMap.put("driveVoltage", driveMotor.getMotorVoltage());
            moduleMap.put("anglePosition", angleMotor.getPosition());
            moduleMap.put("angleVelocity", angleMotor.getVelocity());
            moduleMap.put("angleCurrent", angleMotor.getSupplyCurrent());
            moduleMap.put("angleVoltage", angleMotor.getMotorVoltage());
            moduleMap.put("angleAbsolutePosition", canCoder.getAbsolutePosition());
        }

        Pigeon2 pigeon = this.getPigeon2();
        pigeonYawRate = pigeon.getAngularVelocityZWorld();
        pigeonAccelX = pigeon.getAccelerationX();
        pigeonAccelY = pigeon.getAccelerationY();
        pigeonAccelZ = pigeon.getAccelerationZ();

        BaseStatusSignal.setUpdateFrequencyForAll(100.0, pigeonYawRate, pigeonAccelX, pigeonAccelY,pigeonAccelZ);
    }

    @Override
    public void updateSwerveInputs(SwerveIOInputs inputs) {
        var state = this.getStateCopy();
        inputs.gyroAngle = state.RawHeading;
        inputs.moduleStates = state.ModuleStates;
        inputs.modulePositions = state.ModulePositions;
        inputs.timeStamp = state.Timestamp;
        inputs.estimatedRobotPose = state.Pose;
        inputs.targetStates = state.ModuleTargets;
        inputs.robotRelativeChassisSpeeds = state.Speeds;
        inputs.fieldRelativeChassisSpeeds =
            ChassisSpeeds.fromRobotRelativeSpeeds(state.Speeds, state.Pose.getRotation());

        // gyro rate + body accel — Pi consumes these as Robot/imu (REP-103 units)
        inputs.yawRateRadPerSec = pigeonYawRate.getValue().in(RadiansPerSecond);
        inputs.accelXMetersPerSecSq = pigeonAccelX.getValue().in(MetersPerSecondPerSecond);
        inputs.accelYMetersPerSecSq = pigeonAccelY.getValue().in(MetersPerSecondPerSecond);
        inputs.accelZMetersPerSecSq = (pigeonAccelZ.getValue()).in(MetersPerSecondPerSecond);

    }

    @Override
    public void updateModuleInputs(ModuleIOInputs... inputs) {
        for (int i = 0; i < 4; i++) {
            var moduleMap = swerveModuleSignals.get(i);
            inputs[i].driveId = this.getModule(i).getDriveMotor().getDeviceID();
            inputs[i].drivePosition = moduleMap.get("drivePosition").getValueAsDouble();
            inputs[i].driveVelocity = moduleMap.get("driveVelocity").getValueAsDouble();
            inputs[i].driveAppliedVolts = moduleMap.get("driveVoltage").getValueAsDouble();
            inputs[i].steerId = this.getModule(i).getSteerMotor().getDeviceID();
            inputs[i].steerAbsolutePositionRotations = moduleMap.get("angleAbsolutePosition").getValueAsDouble();
            inputs[i].steerPositionRotations = moduleMap.get("anglePosition").getValueAsDouble();
            inputs[i].steerVelocity = moduleMap.get("angleVelocity").getValueAsDouble();
            inputs[i].steerAppliedVolts = moduleMap.get("angleVoltage").getValueAsDouble();
        }
    }

    @Override
    public Rotation2d getGyro() {
        return Rotation2d.fromDegrees(this.getPigeon2().getYaw().getValueAsDouble());
    }

    @Override
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier, Subsystem requiredSubsystem) {
        return Commands.run(() -> super.setControl(requestSupplier.get()), requiredSubsystem);
    }

    @Override
    public Command setFieldCentric() {
        return Commands.runOnce(() -> super.seedFieldCentric());
    }

    @Override
    public void refreshData() {
        for (int i = 0; i < 4; i++) {
            var moduleMap = swerveModuleSignals.get(i);
            BaseStatusSignal.refreshAll(moduleMap.values().toArray(new BaseStatusSignal[0]));
        }
        BaseStatusSignal.refreshAll(pigeonYawRate, pigeonAccelX, pigeonAccelY,pigeonAccelZ);
    }

    @Override
    public Pose2d getPose() {
        return this.getStateCopy().Pose;
    }

    @Override
    public void addVisionMeasurement(Pose2d visionRobotPose, double timeStamp) {
        super.addVisionMeasurement(visionRobotPose, timeStamp);
    }

    @Override
    public void runCharacterization(SwerveRequest request) {
        setControl(request);
    }

    @Override
    public void driveRequest(SwerveRequest requestSupplier) {
        setControl(requestSupplier);
    }

    @AutoLogOutput
    public ChassisSpeeds getChassisSpeeds() {
        return this.getStateCopy().Speeds;
    }
}
