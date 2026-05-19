package frc.robot.sysid;

import java.util.Set;
import java.util.function.Function;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

/**
 * Routes SysId quasistatic / dynamic commands to whichever subsystem you pick on
 * the Shuffleboard / Elastic dashboard. Put it on Test-mode bindings so it can
 * only run when the DS is in Test.
 *
 * Wire up like:
 * <pre>
 *   selector = new SysIdSelector(swerve, lift, pusher, presser);
 *   testMode.and(operator.a()).whileTrue(selector.quasistatic(Direction.kForward));
 *   testMode.and(operator.b()).whileTrue(selector.quasistatic(Direction.kReverse));
 *   testMode.and(operator.x()).whileTrue(selector.dynamic(Direction.kForward));
 *   testMode.and(operator.y()).whileTrue(selector.dynamic(Direction.kReverse));
 * </pre>
 */
public class SysIdSelector {
    private static final class Entry {
        final Function<Direction, Command> quasi;
        final Function<Direction, Command> dyn;
        Entry(Function<Direction, Command> q, Function<Direction, Command> d) {
            this.quasi = q; this.dyn = d;
        }
    }

    private final SendableChooser<Entry> chooser = new SendableChooser<>();
    private final Set<Subsystem> requirements;

    public SysIdSelector(Set<Subsystem> requirements) {
        this.requirements = requirements;
        SmartDashboard.putData("SysId/Target", chooser);
    }

    public void add(String name,
                    Function<SysIdRoutine.Direction, Command> quasistatic,
                    Function<SysIdRoutine.Direction, Command> dynamic) {
        chooser.addOption(name, new Entry(quasistatic, dynamic));
    }

    public void setDefault(String name,
                           Function<SysIdRoutine.Direction, Command> quasistatic,
                           Function<SysIdRoutine.Direction, Command> dynamic) {
        chooser.setDefaultOption(name, new Entry(quasistatic, dynamic));
    }

    public Command quasistatic(Direction direction) {
        return Commands.defer(() -> {
            Entry selected = chooser.getSelected();
            return selected == null ? Commands.none() : selected.quasi.apply(direction);
        }, requirements);
    }

    public Command dynamic(Direction direction) {
        return Commands.defer(() -> {
            Entry selected = chooser.getSelected();
            return selected == null ? Commands.none() : selected.dyn.apply(direction);
        }, requirements);
    }
}
