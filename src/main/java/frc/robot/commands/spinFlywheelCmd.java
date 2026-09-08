package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.ExampleFlywheel;

/**
 * Spins the example flywheel up to ExampleFlywheel.TARGET_RPM and holds it
 * until the command ends. Paired with ExampleFlywheel, this file shows the
 * smallest complete command: grab a value in initialize(), act every loop in
 * execute(), clean up in end().
 *
 * Note the timing: TARGET_RPM is a live Scope.Tunable, and it is read once in
 * initialize() - so changing it in AdvantageScope tuning mode affects the *next*
 * run of the command, not the one in flight. If you wanted live-while-running
 * behavior you would read it in execute() instead (the flywheel's kP works that
 * way precisely so you can watch gains degrade the graph in real time).
 */
public class spinFlywheelCmd extends Command {
    private final ExampleFlywheel m_flywheel;

    public spinFlywheelCmd(ExampleFlywheel flywheel) {
        m_flywheel = flywheel;
        addRequirements(m_flywheel);
    }

    @Override
    public void initialize() {
        // Read the tunable once per run. Log it too: after a match you can see
        // in the log exactly what target this command was given.
        double target = ExampleFlywheel.TARGET_RPM.get();
        m_flywheel.setTargetRpm(target);
        frc.robot.Library.Scope.put("Flywheel/CommandTarget", target);
    }

    @Override
    public void execute() {
        // Nothing to do every loop: the subsystem's periodic() drives toward the
        // target on its own. A command only needs execute() logic when it has
        // per-loop decisions of its own.
    }

    @Override
    public boolean isFinished() {
        return false; // hold speed until something else (or cancel) ends us
    }

    @Override
    public void end(boolean interrupted) {
        m_flywheel.stop();
    }
}
