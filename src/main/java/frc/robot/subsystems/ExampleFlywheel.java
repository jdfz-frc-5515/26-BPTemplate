package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Library.Scope;

/**
 * A fake flywheel that exists to demonstrate the Scope library
 * (frc.robot.Library.Scope). It has no hardware: a first-order physics model
 * stands in for a real motor, so the logged numbers move around convincingly
 * in simulation or on a robot with no flywheel attached.
 *
 * What to notice (open AdvantageScope while running):
 *  - Scope.put(...)        - live values under "Outputs/Flywheel/..." that are
 *                            also saved to the log file
 *  - Scope.putOnChange(..) - the state enum, graphed as a discrete signal
 *  - Scope.Tunable         - "Tuning/Flywheel/..." entries. Enable tuning mode
 *                            (slider icon) in AdvantageScope and edit them live:
 *                            try lowering kP and watch the RPM curve degrade and
 *                            the "not reaching target" alert appear.
 *
 * On a real robot you would replace the model below with a TalonFX: read its
 * velocity in periodic() and write back the computed voltage. Nothing else
 * would change - the Scope lines stay exactly as they are.
 */
public class ExampleFlywheel extends SubsystemBase {
    // --- Live-tunable numbers (edit from AdvantageScope's tuning mode) ---
    /** Target speed for the spin-up command; the command reads this at initialize(). */
    public static final Scope.Tunable TARGET_RPM = new Scope.Tunable("Flywheel/TargetRpm", 2000);
    /** Volts per RPM of error. Try 0.3 (good), 0.03 (too soft -> alert), 3.0 (slam). */
    private static final Scope.Tunable KP = new Scope.Tunable("Flywheel/kP", 0.3);

    // --- Simulated motor characteristics ---
    private static final double FREE_SPEED_RPM = 4000;    // speed at 12 V
    private static final double TIME_CONSTANT = 0.35;     // spin-up slowness (s)
    private static final double RPM_TOLERANCE = 50;       // "at target" window
    private static final double ALERT_AFTER_SECONDS = 3;  // spin-up time before alerting

    /** Discrete state, graphed in AdvantageScope via Scope.putOnChange. */
    public enum State {
        IDLE,      // target is zero
        SPINNING,  // moving toward a non-zero target
        AT_TARGET  // holding within tolerance
    }

    private State m_state = State.IDLE;
    private double m_targetRpm = 0;
    private double m_rpm = 0;
    private double m_appliedVolts = 0;
    private double m_spinUpSeconds = 0;

    /** Commands the flywheel toward the given speed and holds it there. */
    public void setTargetRpm(double targetRpm) {
        m_targetRpm = targetRpm;
    }

    /** Same as setTargetRpm(0) - reads better at the call site. */
    public void stop() {
        setTargetRpm(0);
    }

    /** True when within RPM_TOLERANCE of the target. */
    public boolean atTarget() {
        return Math.abs(m_rpm - m_targetRpm) <= RPM_TOLERANCE;
    }

    @Override
    public void periodic() {
        // --- Control: P loop on speed error -> volts (real code: motor.setVoltage) ---
        m_appliedVolts = MathUtil.clamp(KP.get() * (m_targetRpm - m_rpm), -12, 12);

        // --- Simulated physics: first-order response toward the free speed ---
        double freeSpeedRpm = m_appliedVolts / 12.0 * FREE_SPEED_RPM;
        m_rpm += (freeSpeedRpm - m_rpm) * (0.02 / TIME_CONSTANT);

        // --- State machine for the discrete graph ---
        if (atTarget()) {
            m_state = State.AT_TARGET;
        } else if (Math.abs(m_targetRpm) > 1) {
            m_state = State.SPINNING;
        } else {
            m_state = State.IDLE;
        }

        // --- Alert: far from target for too long (usually kP tuned too low) ---
        if (!atTarget() && Math.abs(m_targetRpm) > 1) {
            m_spinUpSeconds += 0.02;
        } else {
            m_spinUpSeconds = 0;
        }
        if (m_spinUpSeconds > ALERT_AFTER_SECONDS) {
            Scope.alert("Flywheel/NotReachingTarget",
                "Flywheel stuck " + (int) Math.abs(m_targetRpm - m_rpm) + " RPM from target - tune kP",
                AlertType.kWarning);
        } else {
            Scope.clearAlert("Flywheel/NotReachingTarget");
        }

        // --- Scope logging: one line per signal ---
        Scope.put("Flywheel/Rpm", m_rpm);
        Scope.put("Flywheel/TargetRpm", m_targetRpm);
        Scope.put("Flywheel/AppliedVolts", m_appliedVolts);
        Scope.putOnChange("Flywheel/State", m_state);
    }
}
