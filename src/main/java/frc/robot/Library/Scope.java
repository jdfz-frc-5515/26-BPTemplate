package frc.robot.Library;

import java.util.HashMap;
import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;

/**
 * One-line telemetry + tuning helpers, in the style of SmartDashboard, but built on
 * AdvantageKit's {@link Logger} so that everything is visible live in AdvantageScope
 * AND recorded to the season's .wpilog for post-match review.
 *
 * <p>Why not SmartDashboard: it writes to NetworkTables without recording, so values
 * are lost after the match. Logger.recordOutput appears in the same live sidebar in
 * AdvantageScope, plus it lands in the log file with timestamps — scrub the line
 * graph after a match to see exactly when a sensor flipped or a PID overshot.
 *
 * <p>Typical usage:
 *
 * <pre>{@code
 * // Outputs — one line, like SmartDashboard.putNumber/putBoolean/putString
 * Scope.put("Vision/TagCount", mt2.tagCount);
 * Scope.put("Intake/HasGamePiece", beamBreak.get());
 * Scope.put("Intake/State", state.toString());
 *
 * // Tunables — editable live from AdvantageScope's tuning mode
 * private static final Scope.Tunable kShootSpeed = new Scope.Tunable("Shooter/Speed", 3000);
 * shooter.setVelocity(kShootSpeed.get());
 *
 * // Periodic suppliers — log a lambda's current value every loop without a periodic()
 * Scope.putSupplier("Elevator/Height", () -> elevator.getHeight());
 *
 * // Alerts — show up in dashboards and are logged
 * Scope.alert("Intake/Jam", "Intake current spike", AlertType.kWarning);
 * }</pre>
 *
 * <p>For strings being logged as an enum-like state, prefer logging the enum directly
 * with {@link #put(String, Enum)} so AdvantageScope shows it as a discrete field.
 */
public final class Scope {
    private Scope() {} // static-only utility class

    // ======================= Outputs (one-line puts) =======================

    /** Logs a number under "Outputs/<key>". */
    public static void put(String key, double value) {
        Logger.recordOutput("Outputs/" + key, value);
    }

    /** Logs a boolean under "Outputs/<key>". */
    public static void put(String key, boolean value) {
        Logger.recordOutput("Outputs/" + key, value);
    }

    /** Logs an int under "Outputs/<key>". */
    public static void put(String key, int value) {
        Logger.recordOutput("Outputs/" + key, value);
    }

    /** Logs a string under "Outputs/<key>". */
    public static void put(String key, String value) {
        Logger.recordOutput("Outputs/" + key, value);
    }

    /** Logs an enum's name under "Outputs/<key>" (AdvantageScope renders it as discrete). */
    public static <E extends Enum<E>> void put(String key, E value) {
        Logger.recordOutput("Outputs/" + key, value.name());
    }

    /** Logs an enum only when it changed since the previous call, to reduce log size. */
    public static <E extends Enum<E>> void putOnChange(String key, E value) {
        String last = LAST_VALUES.get(key);
        String current = value.name();
        if (!current.equals(last)) {
            LAST_VALUES.put(key, current);
            Logger.recordOutput("Outputs/" + key, current);
        }
    }

    private static final Map<String, String> LAST_VALUES = new HashMap<>();

    /**
     * Logs the value returned by a supplier every loop. Call from a subsystem's
     * periodic(); the lambda runs once per call, not on a timer of its own.
     */
    public static void putSupplier(String key, DoubleSupplier supplier) {
        put(key, supplier.getAsDouble());
    }

    /** Boolean variant of {@link #putSupplier(String, DoubleSupplier)}. */
    public static void putSupplier(String key, BooleanSupplier supplier) {
        put(key, supplier.getAsBoolean());
    }

    /** String variant of {@link #putSupplier(String, DoubleSupplier)}. */
    public static void putSupplier(String key, Supplier<String> supplier) {
        put(key, supplier.get());
    }

    // ======================= Tunables (live-editable from AdvantageScope) =======================

    /**
     * A number you can edit live from AdvantageScope's tuning mode (fields published to
     * the "/Tuning" table become editable when tuning mode is on). Construct once as a
     * static final field, call {@link #get()} wherever the value is needed.
     *
     * <p>Values set from AdvantageScope persist until the robot code restarts (in
     * keeping with AdvantageKit's LoggedNetworkNumber). On restart the default applies
     * again; tune-then-copy-into-Constants is the intended workflow.
     */
    public static final class Tunable {
        private final LoggedNetworkNumber number;

        /**
         * @param key    name under the "/Tuning" table, e.g. "Shooter/Speed"
         * @param value  default until tuned from AdvantageScope
         */
        public Tunable(String key, double value) {
            number = new LoggedNetworkNumber("/Tuning/" + key, value);
        }

        /** Current value: the tuned value if set from AdvantageScope, else the default. */
        public double get() {
            return number.get();
        }

        /** Change the default (rarely needed; prefer editing the code's default). */
        public void setDefault(double value) {
            number.setDefault(value);
        }
    }

    // ======================= Alerts =======================

    private static final Map<String, Alert> ALERTS = new HashMap<>();

    /**
     * Creates-or-updates a named alert. Active alerts appear on driver dashboards and
     * AdvantageScope (line graph → discrete → "Alerts" group) and are logged.
     *
     * @param key    unique name, e.g. "Intake/Jam"
     * @param text   message shown when active
     * @param type   kInfo / kWarning / kError
     */
    public static void alert(String key, String text, AlertType type) {
        Alert a = ALERTS.computeIfAbsent(typeKey(key, type), k -> new Alert(k, type));
        a.setText(text);
        a.set(true);
        Logger.recordOutput("Outputs/Alerts/" + key, true);
    }

    /** Clears a previously-set alert. No-op if the alert was never set. */
    public static void clearAlert(String key) {
        for (AlertType type : AlertType.values()) {
            Alert a = ALERTS.get(typeKey(key, type));
            if (a != null && a.get()) {
                a.set(false);
                Logger.recordOutput("Outputs/Alerts/" + key, false);
            }
        }
    }

    /** @return true if the alert is currently active (false if never set). */
    public static boolean alertActive(String key) {
        for (AlertType type : AlertType.values()) {
            Alert a = ALERTS.get(typeKey(key, type));
            if (a != null && a.get()) {
                return true;
            }
        }
        return false;
    }

    private static String typeKey(String key, AlertType type) {
        return key + "@" + type.name();
    }
}
