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
import frc.robot.commands.MissionSequences;
import frc.robot.commands.Nav2Drive;
import frc.robot.generated.TunerConstants;
import frc.robot.nt.MissionIO;
import frc.robot.nt.NtCmdVelReceiver;
import frc.robot.nt.NtOdomImuPublisher;
import frc.robot.subsystems.drive.SwerveIOCTRE;
import frc.robot.subsystems.drive.SwerveSubsystem;
import frc.robot.subsystems.lift.LiftSubsystem;
import frc.robot.subsystems.lift.LunchLockSubsystem;
import frc.robot.subsystems.lift.PusherSubsystem;
import frc.robot.subsystems.presser.ButtonPresserSubsystem;
import frc.robot.supervisor.MissionSupervisor;
import frc.robot.sysid.SysIdSelector;

public class RobotContainer {
    // tuning knobs
    private static final double LIFT_JOG_SCALE = 0.4;     // operator |stick| -> % output
    private static final double PUSHER_JOG_SCALE = 0.4;
    private static final double PRESSER_JOG_SCALE = 0.3;  // held-button % output
    private static final double STICK_DEADBAND = 0.1;

    // operator preset positions, in leader-motor rotations (TUNE)
    private static final double LIFT_HOME_ROT = 0.0;
    private static final double LIFT_DELIVER_ROT = 8.0;
    private static final double PUSHER_RETRACTED_ROT = 0.0;
    private static final double PUSHER_EXTENDED_ROT = 5.0;
    private static final double PRESSER_TEST_ROT = 20.0; // bench press-test height

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
    // Soft-limit ranges in leader-motor rotations (TUNE — measure your travel).
    // Each mechanism's encoder zeros at boot, so it MUST start at home (0).
    private static final double LIFT_MIN_ROT = 12, LIFT_MAX_ROT = 44.80;
    private static final double PUSHER_MIN_ROT = 0.3, PUSHER_MAX_ROT = 38.550;
    private static final double PRESSER_MIN_ROT = 3.0, PRESSER_MAX_ROT = 54.94;

    // Lunch lift: 2 NEO Vortex (1 each side, follower inverted)
    private final LiftSubsystem lift = new LiftSubsystem(
        "Lift",
        /* leader */ 3,
        /* followers */ new int[] { 4 },
        /* followerInverted */ new boolean[] { true },
        /* kP */ 0.05,
        LIFT_MIN_ROT, LIFT_MAX_ROT);

    // Horizontal pusher on top of the lift: 2 NEO Vortex (leader + inverted follower)
    private final PusherSubsystem pusher = new PusherSubsystem(
        /* leader */ 1,
        /* follower */ 2,
        /* followerInverted */ true,
        /* kP */ 0.05,
        PUSHER_MIN_ROT, PUSHER_MAX_ROT);

    // Button-press arm: 1 NEO Vortex (vertical lift) + REV PH pneumatic poker on top
    private final ButtonPresserSubsystem presser = new ButtonPresserSubsystem(
        /* canId */ 5,
        /* kP */ 0.05,
        PRESSER_MIN_ROT, PRESSER_MAX_ROT,
        /* phModuleId */ 2,
        /* solenoidChan */ 2);

    // Two single-acting lunch locks. energizedToLock flags set OPPOSITE as a
    // starting guess for "one opens, one closes" — verify on the bench and flip
    // whichever is backwards. Channels are placeholders.
    private final LunchLockSubsystem lunchLocks = new LunchLockSubsystem(
        /* phModuleId */ 2,
        /* robotLockChan */ 0, /* robotEnergizedToLock */ true,
        /* transferLockChan */ 1, /* transferEnergizedToLock */ false);

    private final MissionSupervisor missionSupervisor = new MissionSupervisor(
        presser, lift, pusher, lunchLocks, missionIO,
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
        Trigger teleop = RobotModeTriggers.teleop();
        Trigger test = RobotModeTriggers.test();

        // ================= DRIVER (port 0) =================
        // swerve
        driver.leftBumper().onTrue(swerve.setFieldCentric());

        // right trigger = "Pi is driving": Nav2Drive owns the swerve (sticks locked
        // out) and enable=true. Release -> teleop sticks resume, enable=false.
        driver.rightTrigger(0.5).whileTrue(nav2Drive);
        driver.rightTrigger(0.5).onTrue(Commands.runOnce(() -> missionIO.setEnable(true)));
        driver.rightTrigger(0.5).onFalse(Commands.runOnce(() -> missionIO.setEnable(false)));

        // restart/abort
        driver.back().onTrue(Commands.runOnce(() -> {
            missionIO.fireMissionRestart();
            missionSupervisor.restart();
        }));

        // ---- DRIVER face buttons: full-sequence bench tests ----
        // These schedule the sequences directly, bypassing the Pi + dead-man, so
        // you can dry-run the whole motion on the bench. Suppliers fake the lunch
        // sensor so the sequence doesn't stall waiting on it.
        driver.a().onTrue(MissionSequences.load(lift, pusher, lunchLocks, missionIO, () -> true));
        driver.b().onTrue(MissionSequences.unload(lift, pusher, lunchLocks, missionIO, () -> false));
        driver.x().onTrue(MissionSequences.stow(lift, pusher, lunchLocks));
        driver.y().onTrue(pressTest());

        // ================= OPERATOR (port 1) =================
        // (defaults: left Y jogs lift, right Y jogs pusher — set in constructor)

        // ---- lift / pusher presets (teleop only, so they don't clash with SysId
        //      which reuses A/B/X/Y in Test mode) ----
        teleop.and(operator.a()).onTrue(Commands.runOnce(() -> lift.setPosition(LIFT_HOME_ROT), lift));
        teleop.and(operator.b()).onTrue(Commands.runOnce(() -> lift.setPosition(LIFT_DELIVER_ROT), lift));
        teleop.and(operator.x()).onTrue(Commands.runOnce(() -> pusher.setPosition(PUSHER_RETRACTED_ROT), pusher));
        teleop.and(operator.y()).onTrue(Commands.runOnce(() -> pusher.setPosition(PUSHER_EXTENDED_ROT), pusher));

        // ---- presser lift jog (hold) + poker ----
        operator.leftBumper().whileTrue(Commands.run(() -> presser.setOpenLoop(PRESSER_JOG_SCALE), presser));
        operator.leftBumper().onFalse(Commands.runOnce(presser::stopLift, presser));
        operator.leftTrigger(0.5).whileTrue(Commands.run(() -> presser.setOpenLoop(-PRESSER_JOG_SCALE), presser));
        operator.leftTrigger(0.5).onFalse(Commands.runOnce(presser::stopLift, presser));
        operator.rightBumper().onTrue(Commands.runOnce(presser::extendPoker));
        operator.rightBumper().onFalse(Commands.runOnce(presser::retractPoker));

        // ---- lunch locks (D-pad): up=lock robot, down=unlock robot,
        //      left=lock transfer, right=unlock transfer ----
        operator.povUp().onTrue(Commands.runOnce(lunchLocks::lockRobot));
        operator.povDown().onTrue(Commands.runOnce(lunchLocks::unlockRobot));
        operator.povLeft().onTrue(Commands.runOnce(lunchLocks::lockTransfer));
        operator.povRight().onTrue(Commands.runOnce(lunchLocks::unlockTransfer));

        // ================= SysId (Test mode only) =================
        // Pick the target on the "SysId/Target" chooser (Shuffleboard/Elastic).
        test.and(operator.a()).whileTrue(sysId.quasistatic(SysIdRoutine.Direction.kForward));
        test.and(operator.b()).whileTrue(sysId.quasistatic(SysIdRoutine.Direction.kReverse));
        test.and(operator.x()).whileTrue(sysId.dynamic(SysIdRoutine.Direction.kForward));
        test.and(operator.y()).whileTrue(sysId.dynamic(SysIdRoutine.Direction.kReverse));
    }

    /** Full presser cycle for the bench: lift to height, poke, retract, return home. */
    private Command pressTest() {
        return Commands.sequence(
            Commands.runOnce(presser::retractPoker),
            Commands.runOnce(() -> presser.setLiftPosition(PRESSER_TEST_ROT), presser),
            Commands.waitUntil(() -> presser.liftAtPosition(0.5)),
            Commands.runOnce(presser::extendPoker),
            Commands.waitSeconds(0.5),
            Commands.runOnce(presser::retractPoker),
            Commands.waitSeconds(0.3),
            Commands.runOnce(() -> presser.setLiftPosition(0.0), presser),
            Commands.waitUntil(() -> presser.liftAtPosition(0.5))
        ).withName("PressTest");
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
