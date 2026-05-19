package frc.robot.subsystems.lift;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Two single-acting (spring-return) pneumatic locks for the lunch:
 *
 *  - robotLock    — secures the lunch inside the robot once it's loaded.
 *  - transferLock — grips the lunch during the push/pull so it can't fall off
 *                   while the pusher moves it in or out.
 *
 * Each lock can be energize-to-lock OR energize-to-unlock (spring does the other
 * direction) — pass the {@code energizedToLock} flag per lock so lock()/unlock()
 * map to the right solenoid state. That's how we handle "one opens, one closes"
 * at rest: set their flags opposite.
 */
public class LunchLockSubsystem extends SubsystemBase {
    private final Solenoid robotLock;
    private final Solenoid transferLock;
    private final boolean robotEnergizedToLock;
    private final boolean transferEnergizedToLock;

    private boolean robotLocked;
    private boolean transferLocked;

    /**
     * @param phModuleId              CAN ID of the REV PH
     * @param robotLockChan           PH channel for the in-robot lock
     * @param robotEnergizedToLock    true if energizing the solenoid LOCKS it
     * @param transferLockChan        PH channel for the transfer gripper lock
     * @param transferEnergizedToLock true if energizing the solenoid LOCKS it
     */
    public LunchLockSubsystem(int phModuleId,
                              int robotLockChan, boolean robotEnergizedToLock,
                              int transferLockChan, boolean transferEnergizedToLock) {
        this.robotEnergizedToLock = robotEnergizedToLock;
        this.transferEnergizedToLock = transferEnergizedToLock;
        robotLock = new Solenoid(phModuleId, PneumaticsModuleType.REVPH, robotLockChan);
        transferLock = new Solenoid(phModuleId, PneumaticsModuleType.REVPH, transferLockChan);
        unlockRobot();
        unlockTransfer();
    }

    public void lockRobot() {
        robotLocked = true;
        robotLock.set(robotEnergizedToLock);
    }

    public void unlockRobot() {
        robotLocked = false;
        robotLock.set(!robotEnergizedToLock);
    }

    public void lockTransfer() {
        transferLocked = true;
        transferLock.set(transferEnergizedToLock);
    }

    public void unlockTransfer() {
        transferLocked = false;
        transferLock.set(!transferEnergizedToLock);
    }

    public boolean isRobotLocked() {
        return robotLocked;
    }

    public boolean isTransferLocked() {
        return transferLocked;
    }

    @Override
    public void periodic() {
        Logger.recordOutput("LunchLock/robotLocked", robotLocked);
        Logger.recordOutput("LunchLock/transferLocked", transferLocked);
    }
}
