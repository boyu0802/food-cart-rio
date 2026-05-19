package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.RobotCentric;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.nt.MissionIO;
import frc.robot.nt.NtCmdVelReceiver;
import frc.robot.subsystems.drive.SwerveSubsystem;

/**
 * Drives the swerve from Nav/cmd published by the Pi nt_bridge (Nav2 / cmd_vel).
 * Robot-centric — the Pi sends body-frame velocities.
 *
 * Two safety gates:
 *  - if the Pi heartbeat stops for {@code NAV_CMD_MAX_AGE_SEC}, sends zeros;
 *  - if the operator dead-man (Robot/mission/enable) isn't held, sends zeros.
 */
public class Nav2Drive extends Command {
    private final SwerveSubsystem swerve;
    private final NtCmdVelReceiver cmd;
    private final MissionIO missionIo;

    private final SwerveRequest.RobotCentric request = new RobotCentric()
        .withDriveRequestType(DriveRequestType.Velocity)
        .withDeadband(0.05)
        .withRotationalDeadband(0.01);

    public Nav2Drive(SwerveSubsystem swerve, NtCmdVelReceiver cmd, MissionIO missionIo) {
        this.swerve = swerve;
        this.cmd = cmd;
        this.missionIo = missionIo;
        addRequirements(swerve);
    }

    @Override
    public void execute() {
        cmd.poll();
        boolean gated = !missionIo.isEnabled();
        swerve.driveRequest(
            request
                .withVelocityX(gated ? 0 : cmd.vx())
                .withVelocityY(gated ? 0 : cmd.vy())
                .withRotationalRate(gated ? 0 : cmd.omega()));
    }

    @Override
    public void end(boolean interrupted) {
        swerve.driveRequest(
            request.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
    }
}
