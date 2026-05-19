// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentric;

import java.util.Set;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.Nav2Drive;
import frc.robot.generated.TunerConstants;
import frc.robot.nt.MissionIO;
import frc.robot.nt.NtCmdVelReceiver;
import frc.robot.nt.NtOdomImuPublisher;
import frc.robot.subsystems.drive.SwerveIOCTRE;
import frc.robot.subsystems.drive.SwerveSubsystem;
import frc.robot.subsystems.lift.LiftSubsystem;
import frc.robot.subsystems.lift.PusherSubsystem;
import frc.robot.subsystems.presser.ButtonPresserSubsystem;
import frc.robot.supervisor.MissionSupervisor;
import frc.robot.sysid.SysIdSelector;

public class RobotContainer {
    // tuning knobs
    private static final double LIFT_JOG_SCALE = 0.4;     // operator |stick| -> % output
    private static final double PUSHER_JOG_SCALE = 0.4;
    private static final double STICK_DEADBAND = 0.1;

    // operator preset positions, in leader-motor rotations (TUNE)
    private static final double LIFT_HOME_ROT = 0.0;
    private static final double LIFT_DELIVER_ROT = 8.0;
    private static final double PUSHER_RETRACTED_ROT = 0.0;
    private static final double PUSHER_EXTENDED_ROT = 5.0;

    // === driver / operator inputs ===
    private final CommandXboxController driver = new CommandXboxController(0);
    private final CommandXboxController operator = new CommandXboxController(1);

    // === drivetrain ===
    private final SwerveSubsystem swerve = new SwerveSubsystem(
        new SwerveIOCTRE(
            TunerConstants.DrivetrainConstants,
            TunerConstants.FrontLeft,
            TunerConstants.BackLeft,
            TunerConstants.BackRight,
            TunerConstants.FrontRight));

    // === NT contract with the Pi nt_bridge ===
    private final NtCmdVelReceiver navCmd = new NtCmdVelReceiver();
    private final NtOdomImuPublisher ntPublisher = new NtOdomImuPublisher(swerve);
    private final MissionIO missionIO = new MissionIO();

    // === cart mechanisms ===
    // Lunch lift: 2 NEO Vortex (1 each side, follower inverted)
    private final LiftSubsystem lift = new LiftSubsystem(
        "Lift",
        /* leader */ 20,
        /* followers */ new int[] { 21 },
        /* followerInverted */ new boolean[] { true },
        /* kP */ 0.05);

    // Horizontal pusher on top of the lift: 1 NEO Vortex
    private final PusherSubsystem pusher = new PusherSubsystem(/* canId */ 22, /* kP */ 0.05);

    // Button-press arm: 1 NEO Vortex (vertical lift) + REV PH pneumatic poker on top
    private final ButtonPresserSubsystem presser = new ButtonPresserSubsystem(
        /* canId */ 23,
        /* kP */ 0.05,
        /* phModuleId */ 1,
        /* solenoidChan */ 0);

    private final MissionSupervisor missionSupervisor = new MissionSupervisor(
        presser, lift, pusher, missionIO,
        /* startDio */ -1,
        /* loadedDio */ -1);

    private final Compressor compressor = new Compressor(/* phModuleId */ 1, PneumaticsModuleType.REVPH);

    // === SysId selector — choose target on the dashboard, run via test-mode buttons ===
    private final SysIdSelector sysId = new SysIdSelector(
        Set.<Subsystem>of(swerve, lift, pusher, presser));

    // === commands ===
    private final Nav2Drive nav2Drive = new Nav2Drive(swerve, navCmd, missionIO);

    private final SwerveRequest.FieldCentric teleopRequest = new FieldCentric()
        .withDriveRequestType(DriveRequestType.Velocity);

    public RobotContainer() {
        swerve.setDefaultCommand(
            swerve.applyRequest(() -> teleopRequest
                .withVelocityX(-driver.getLeftY())
                .withVelocityY(-driver.getLeftX())
                .withRotationalRate(-driver.getRightX())));

        // Default: operator left Y jogs the lift, right Y jogs the pusher.
        // Brake mode holds them in place when sticks are neutral.
        lift.setDefaultCommand(Commands.run(
            () -> lift.setOpenLoop(jogValue(-operator.getLeftY(), LIFT_JOG_SCALE)),
            lift));
        pusher.setDefaultCommand(Commands.run(
            () -> pusher.setOpenLoop(jogValue(-operator.getRightY(), PUSHER_JOG_SCALE)),
            pusher));

        // missionSupervisor is constructed for its side effects
        // (CommandScheduler auto-registration).
        if (missionSupervisor == null) throw new IllegalStateException();

        compressor.enableDigital();

        configureSysIdTargets();
        configureBindings();
    }

    private void configureSysIdTargets() {
        sysId.setDefault("Swerve Translation",
            swerve::sysIdTranslationQuasistatic, swerve::sysIdTranslationDynamic);
        sysId.add("Swerve Steer",
            swerve::sysIdSteerQuasistatic, swerve::sysIdSteerDynamic);
        sysId.add("Lift",     lift::sysIdQuasistatic,    lift::sysIdDynamic);
        sysId.add("Pusher",   pusher::sysIdQuasistatic,  pusher::sysIdDynamic);
        sysId.add("Presser",  presser::sysIdQuasistatic, presser::sysIdDynamic);
    }

    private static double jogValue(double raw, double scale) {
        return MathUtil.applyDeadband(raw, STICK_DEADBAND) * scale;
    }

    private void configureBindings() {
        // ---- driver: swerve ----
        driver.leftBumper().onTrue(swerve.setFieldCentric());

        // ---- right trigger = "Pi is driving" ----
        // Held:
        //   - Nav2Drive owns the swerve (default teleop is locked out by the
        //     subsystem requirement, so sticks have no effect)
        //   - enable=true is published, so mission sequences + press machine advance
        // Released:
        //   - Nav2Drive ends, default teleop resumes, sticks work normally
        //   - enable=false is published, so sequences freeze / cancel
        driver.rightTrigger(0.5).whileTrue(nav2Drive);
        driver.rightTrigger(0.5).onTrue(Commands.runOnce(() -> missionIO.setEnable(true)));
        driver.rightTrigger(0.5).onFalse(Commands.runOnce(() -> missionIO.setEnable(false)));

        // ---- restart/abort: cancel sequences, return everything to IDLE,
        // pulse Robot/mission/restart_pressed so the Pi can drop its plan too.
        driver.back().onTrue(Commands.runOnce(() -> {
            missionIO.fireMissionRestart();
            missionSupervisor.restart();
        }));

        // ---- operator: lift presets ----
        operator.a().onTrue(Commands.runOnce(() -> lift.setPosition(LIFT_HOME_ROT), lift));
        operator.b().onTrue(Commands.runOnce(() -> lift.setPosition(LIFT_DELIVER_ROT), lift));

        // ---- operator: pusher presets ----
        operator.x().onTrue(Commands.runOnce(() -> pusher.setPosition(PUSHER_RETRACTED_ROT), pusher));
        operator.y().onTrue(Commands.runOnce(() -> pusher.setPosition(PUSHER_EXTENDED_ROT), pusher));

        // ---- operator: manual poker fire (for testing the presser arm without Pi) ----
        operator.rightBumper().onTrue(Commands.runOnce(presser::extendPoker));
        operator.rightBumper().onFalse(Commands.runOnce(presser::retractPoker));

        // ---- SysId (Test mode on the Driver Station only) ----
        // Pick the target on Shuffleboard/Elastic → "SysId/Target" chooser.
        Trigger test = RobotModeTriggers.test();
        test.and(operator.a()).whileTrue(sysId.quasistatic(SysIdRoutine.Direction.kForward));
        test.and(operator.b()).whileTrue(sysId.quasistatic(SysIdRoutine.Direction.kReverse));
        test.and(operator.x()).whileTrue(sysId.dynamic(SysIdRoutine.Direction.kForward));
        test.and(operator.y()).whileTrue(sysId.dynamic(SysIdRoutine.Direction.kReverse));
    }

    /** Called once per main loop tick from Robot.robotPeriodic. */
    public void tickPeriodic() {
        ntPublisher.publishOdom();
    }

    public Command getAutonomousCommand() {
        // In auto, just follow whatever the Pi is publishing on Nav/cmd.
        return new Nav2Drive(swerve, navCmd, missionIO);
    }
}
