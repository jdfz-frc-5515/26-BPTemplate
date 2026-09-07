package frc.robot;

import edu.wpi.first.math.util.Units;

/**
 * Central place for every tunable number on the robot.
 * Grouped into nested classes by subsystem / concern.
 */
public class Constants {

    /** Gains and limits for the "move to pose" controllers on the drivetrain. */
    public static final class AutoConstants {
        // Move-to-pose translation PID (field-frame X and Y each get their own controller)
        public static final double moveToPoseTranslationkP = 3;
        public static final double moveToPoseTranslationkI = 0.;
        public static final double moveToPoseTranslationkD = 0.;

        // Move-to-pose rotation PID
        public static final double moveToPoseRotationkP = 5;
        public static final double moveToPoseRotationkI = 2.;
        public static final double moveToPoseRotationkD = 0.;

        // Output clamps applied after the PID loops
        public static final double maxMoveToSpeed = 3; // m/s
        public static final double maxMoveToAngularVelocity = Units.degreesToRadians(200.);

        // Outputs smaller than these are treated as zero (prevents jitter around the setpoint)
        public static final double moveToVelocityDeadband = 0.01;
        public static final double moveToAnguVeloDeadband = 0.001;
    }

    /** Vision (Limelight) pose-estimation settings, used by CommandSwerveDrivetrain. */
    public static final class VisionConstants {
        // NetworkTables name of your Limelight. Change this to match your camera.
        public static final String LIMELIGHT_NAME = "limelight";

        // --- MegaTag2 (mt2) fusion filters: an update is only fused if ALL pass ---
        public static final double MAX_LATENCY_MS = 100.0;                    // reject stale frames
        public static final double MAX_ANGULAR_VELOCITY_RADS = 4.0 * Math.PI; // spinning breaks MT2
        public static final double MAX_LINEAR_VELOCITY_MPS = 2.0;             // fast driving blurs tags
        public static final double MAX_TAG_DISTANCE_METERS = 4.0;             // distant tags are noisy
        public static final double STD_DEV_BASE = 0.1;                        // XY uncertainty @ 1m, scales with dist^2

        // --- Gyro reseed from MegaTag1 (mt1): only when very confident ---
        public static final int RESEED_MIN_TAG_COUNT = 2;
        public static final double RESEED_MAX_TAG_DISTANCE = 3.0;
        public static final double RESEED_MAX_OMEGA_RADS = 0.2;
        public static final double RESEED_DISCREPANCY_DEG = 1.0;              // only reseed if off by more than this
    }
}
