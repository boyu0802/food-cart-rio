package frc.robot.nt;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Notifier;
import frc.robot.subsystems.drive.SwerveSubsystem;

/**
 * Publishes the rio-side half of the nt_bridge contract:
 *   Robot/odom/{x,y,theta,vx,vy,omega}  @ robot loop (50 Hz)
 *   Robot/imu/{yaw,yaw_rate,accel_x,accel_y}  @ 100 Hz via Notifier
 *
 * Frame convention: ROS2 REP-103 — x forward, y left, yaw CCW positive.
 * vx/vy/omega in Robot/odom are body-frame (the contract calls them out as such).
 */
public class NtOdomImuPublisher {
    private final SwerveSubsystem swerve;

    private final DoublePublisher odomX;
    private final DoublePublisher odomY;
    private final DoublePublisher odomTheta;
    private final DoublePublisher odomVx;
    private final DoublePublisher odomVy;
    private final DoublePublisher odomOmega;

    private final DoublePublisher imuYaw;
    private final DoublePublisher imuYawRate;
    private final DoublePublisher imuAccelX;
    private final DoublePublisher imuAccelY;
    private final DoublePublisher imuAccelZ;


    private final Notifier imuNotifier;

    public NtOdomImuPublisher(SwerveSubsystem swerve) {
        this(swerve, NetworkTableInstance.getDefault());
    }

    public NtOdomImuPublisher(SwerveSubsystem swerve, NetworkTableInstance inst) {
        this.swerve = swerve;
        NetworkTable odom = inst.getTable(NtContract.ROBOT_ODOM_TABLE);
        NetworkTable imu = inst.getTable(NtContract.ROBOT_IMU_TABLE);

        odomX = odom.getDoubleTopic(NtContract.ODOM_X).publish();
        odomY = odom.getDoubleTopic(NtContract.ODOM_Y).publish();
        odomTheta = odom.getDoubleTopic(NtContract.ODOM_THETA).publish();
        odomVx = odom.getDoubleTopic(NtContract.ODOM_VX).publish();
        odomVy = odom.getDoubleTopic(NtContract.ODOM_VY).publish();
        odomOmega = odom.getDoubleTopic(NtContract.ODOM_OMEGA).publish();

        imuYaw = imu.getDoubleTopic(NtContract.IMU_YAW).publish();
        imuYawRate = imu.getDoubleTopic(NtContract.IMU_YAW_RATE).publish();
        imuAccelX = imu.getDoubleTopic(NtContract.IMU_ACCEL_X).publish();
        imuAccelY = imu.getDoubleTopic(NtContract.IMU_ACCEL_Y).publish();
        imuAccelZ = imu.getDoubleTopic(NtContract.IMU_ACCEL_Z).publish();

        imuNotifier = new Notifier(this::publishImu);
        imuNotifier.setName("NtImuPublisher");
        imuNotifier.startPeriodic(1.0 / NtContract.IMU_PUBLISH_HZ);
    }

    /** Call from SwerveSubsystem.periodic() or RobotContainer @ 50 Hz. */
    public void publishOdom() {
        Pose2d pose = swerve.getPose();
        ChassisSpeeds speeds = swerve.getRobotRelativeSpeeds();
        odomX.set(pose.getX());
        odomY.set(pose.getY());
        odomTheta.set(pose.getRotation().getRadians());
        odomVx.set(speeds.vxMetersPerSecond);
        odomVy.set(speeds.vyMetersPerSecond);
        odomOmega.set(speeds.omegaRadiansPerSecond);
    }

    private void publishImu() {
        imuYaw.set(swerve.getYawRadians());
        imuYawRate.set(swerve.getYawRateRadPerSec());
        imuAccelX.set(swerve.getAccelXMetersPerSecSq());
        imuAccelY.set(swerve.getAccelYMetersPerSecSq());
        imuAccelZ.set(swerve.getAccelZMetersPerSecSq());
    }

    public void close() {
        imuNotifier.stop();
        imuNotifier.close();
    }
}
