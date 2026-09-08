package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.Library.Scope;
import frc.robot.generated.TunerConstants;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;
import frc.robot.Library.ImprovedCommandXboxController;
import frc.robot.LimelightHelpers;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.
 *
 * Bundles what a swerve drive typically needs over a season:
 *  - Manual field-centric driving from an Xbox controller (with an expo curve)
 *  - "Move to pose" PID control, plus a hybrid mode where the driver can nudge the target
 *  - PathPlanner auto / pathfinding support
 *  - Limelight MegaTag2 vision fusion + gyro reseed from MegaTag1
 */
public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    /* Blue alliance perspective: forward is 0 degrees (toward red alliance wall) */
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    /* Red alliance perspective: forward is 180 degrees (toward blue alliance wall) */
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean m_hasAppliedOperatorPerspective = false;

    /** Swerve request to apply during field-centric move to Pose */
    private static final SwerveRequest.ApplyFieldSpeeds m_moveToPoseDrive = new SwerveRequest.ApplyFieldSpeeds().withDriveRequestType(DriveRequestType.Velocity);

    /** Swerve request to apply during robot-centric path following */
    private final SwerveRequest.ApplyRobotSpeeds m_autoDrive = new SwerveRequest.ApplyRobotSpeeds().withDriveRequestType(DriveRequestType.Velocity);

    // --- Move-to-pose PID controllers (X and Y in field frame, plus rotation) ---
    // Gains are Scope.Tunables: edit them live from AdvantageScope's tuning mode
    // ("Tuning" table), then copy the winning values back into Constants.AutoConstants.
    private static final Scope.Tunable kMoveToPoseTranslationP = new Scope.Tunable("Drivetrain/MoveToPose/TranslationP", AutoConstants.moveToPoseTranslationkP);
    private static final Scope.Tunable kMoveToPoseRotationP = new Scope.Tunable("Drivetrain/MoveToPose/RotationP", AutoConstants.moveToPoseRotationkP);
    private static final Scope.Tunable kMoveToPoseRotationI = new Scope.Tunable("Drivetrain/MoveToPose/RotationI", AutoConstants.moveToPoseRotationkI);
    PIDController m_translationXController = new PIDController(kMoveToPoseTranslationP.get(), AutoConstants.moveToPoseTranslationkI, AutoConstants.moveToPoseTranslationkD);
    PIDController m_translationYController = new PIDController(kMoveToPoseTranslationP.get(), AutoConstants.moveToPoseTranslationkI, AutoConstants.moveToPoseTranslationkD);
    PIDController m_rotationController = new PIDController(kMoveToPoseRotationP.get(), kMoveToPoseRotationI.get(), AutoConstants.moveToPoseRotationkD);
    Pose2d m_targetPose2d = new Pose2d();
    double m_distFromTarget = Double.MAX_VALUE;
    double m_angleDistFromTarget = Double.MAX_VALUE;

    // --- Manual driving limits ---
    private static double manual_MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // top speed at 12 V
    private static double manual_MaxAngularRate = RotationsPerSecond.of(1.).in(RadiansPerSecond); // max angular velocity
    private static final SwerveRequest.FieldCentric m_manualDrive = new SwerveRequest.FieldCentric().withDeadband(0.0).withRotationalDeadband(0.0).withDriveRequestType(DriveRequestType.Velocity);

    // Heading offset between the Pigeon yaw and odometry heading. Set by
    // resetHeadingForOdo() and used by the Limelight MegaTag2 pipeline.
    private double zeroOdoDegree = 0;

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants   Drivetrain-wide constants for the swerve drive
     * @param modules               Constants for each specific module
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants     Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency The frequency to run the odometry loop. If
     *                                unspecified or set to 0 Hz, this is 250 Hz on
     *                                CAN FD, and 100 Hz on CAN 2.0.
     * @param modules                 Constants for each specific module
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants       Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency   The frequency to run the odometry loop. If
     *                                  unspecified or set to 0 Hz, this is 250 Hz on
     *                                  CAN FD, and 100 Hz on CAN 2.0.
     * @param odometryStandardDeviation The standard deviation for odometry calculation
     *                                  in the form [x, y, theta], with units in meters
     *                                  and radians
     * @param visionStandardDeviation   The standard deviation for vision calculation
     *                                  in the form [x, y, theta], with units in meters
     *                                  and radians
     * @param modules                   Constants for each specific module
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        Matrix<N3, N1> odometryStandardDeviation,
        Matrix<N3, N1> visionStandardDeviation,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    /**
     * Returns a command that applies the specified control request to this swerve drivetrain.
     *
     * @param requestSupplier Function returning the request to apply
     * @return Command to run
     */
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    @Override
    public void periodic() {
        /*
         * Periodically try to apply the operator perspective.
         * If we haven't applied the operator perspective before, then we should apply it regardless of DS state.
         * This allows us to correct the perspective in case the robot code restarts mid-match.
         * Otherwise, only check and apply the operator perspective if the DS is disabled.
         * This ensures driving behavior doesn't change until an explicit disable event occurs during testing.
         */
        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                setOperatorPerspectiveForward(
                    allianceColor == Alliance.Red
                        ? kRedAlliancePerspectiveRotation
                        : kBlueAlliancePerspectiveRotation
                );
                m_hasAppliedOperatorPerspective = true;
            });
        }
        updateOdometry(VisionConstants.LIMELIGHT_NAME, -1); // -1 disables the max-tag-distance filter
        reseedGyroByVision();

        // Apply tuned PID gains every loop so AdvantageScope tuning-mode edits take
        // effect immediately (cheap calls; the Tunables read from NetworkTables).
        m_translationXController.setP(kMoveToPoseTranslationP.get());
        m_translationYController.setP(kMoveToPoseTranslationP.get());
        m_rotationController.setP(kMoveToPoseRotationP.get());
        m_rotationController.setI(kMoveToPoseRotationI.get());
    }

    // ======================= Vision pose estimation =======================

    /**
     * Sends robot orientation to the Limelight, reads its MegaTag2 estimate and
     * fuses it into the pose estimate when the update passes every quality filter.
     *
     * @param llName  NetworkTables name of the Limelight, e.g. "limelight"
     * @param minDist Reject updates whose average tag distance exceeds this (meters).
     *                Pass -1 to disable the filter.
     * @return average tag distance of the accepted update, or -1 if nothing was fused
     */
    public double updateOdometry(String llName, double minDist) {
        double yaw = getRotationFromPigeon() - zeroOdoDegree;
        double pitch = getPigeon2().getPitch().getValueAsDouble() % 360;
        double roll = getPigeon2().getRoll().getValueAsDouble() % 360;
        LimelightHelpers.SetRobotOrientation(llName, yaw, 0, pitch, 0, roll, 0);

        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(llName);
        if (mt2 == null) {
            Scope.put("Vision/ValidUpdate", false);
            return -1;
        }

        if (minDist > 0 && mt2.avgTagDist > minDist) {
            Scope.put("Vision/ValidUpdate", false);
            return -1;
        }

        double currentSpeed = Math.hypot(getSpeeds().vxMetersPerSecond, getSpeeds().vyMetersPerSecond);

        // Filter chain: reject vision updates while spinning/driving fast,
        // or when tags are far away or the frame is stale.
        if (Math.abs(getSpeeds().omegaRadiansPerSecond) <= VisionConstants.MAX_ANGULAR_VELOCITY_RADS
                && mt2.tagCount > 0
                && mt2.avgTagDist < VisionConstants.MAX_TAG_DISTANCE_METERS
                && currentSpeed < VisionConstants.MAX_LINEAR_VELOCITY_MPS
                && mt2.latency < VisionConstants.MAX_LATENCY_MS) {
            double captureTime2 = Utils.fpgaToCurrentTime(mt2.timestampSeconds);

            // 2+ tags means stronger geometry, so trust the estimate more (smaller std dev)
            double trustFactor = mt2.tagCount >= 2 ? 0.5 : 1.0;
            double xyStdDev = VisionConstants.STD_DEV_BASE * Math.pow(mt2.avgTagDist, 2) * trustFactor;
            double rotStdDev = 999999.0; // MegaTag2 is translation-only, trust gyro for rotation

            addVisionMeasurement(mt2.pose, captureTime2, VecBuilder.fill(xyStdDev, xyStdDev, rotStdDev));

            Scope.put("Vision/ValidUpdate", true);
            Scope.put("Vision/AvgTagDist", mt2.avgTagDist);
            Scope.put("Vision/TagCount", mt2.tagCount);
            Scope.put("Vision/LatencyMs", mt2.latency);

            return mt2.avgTagDist;
        } else {
            Scope.put("Vision/ValidUpdate", false);
        }
        return -1;
    }

    /**
     * MegaTag2 cannot estimate heading, so the gyro is the only rotation source.
     * This runs MegaTag1 alongside; when MT1 is very confident (several close tags,
     * robot nearly still) and disagrees with the gyro by more than the threshold,
     * we re-zero the odometry heading from vision. Fixes gyro drift in long matches.
     */
    public void reseedGyroByVision() {
        LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(VisionConstants.LIMELIGHT_NAME);

        if (mt1 != null && mt1.tagCount >= VisionConstants.RESEED_MIN_TAG_COUNT && mt1.avgTagDist < VisionConstants.RESEED_MAX_TAG_DISTANCE) {
            if (Math.abs(getSpeeds().omegaRadiansPerSecond) < VisionConstants.RESEED_MAX_OMEGA_RADS) {
                Rotation2d targetRot = mt1.pose.getRotation();
                double newDeg = targetRot.getDegrees();
                double curDeg = getRotationFromPigeon() - zeroOdoDegree;

                // Only reseed if there is a significant discrepancy (> threshold)
                if (Math.abs(newDeg - curDeg) > VisionConstants.RESEED_DISCREPANCY_DEG) {
                    zeroOdoDegree -= (newDeg - curDeg);
                    resetPose(new Pose2d(getState().Pose.getTranslation(), targetRot));

                    Scope.put("Vision/LastReseedDeg", newDeg);
                    Scope.put("Vision/LastReseedDiscrepancy", newDeg - curDeg);
                }
            }
        }
    }

    // ======================= Simulation =======================

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    // ======================= State accessors =======================

    public Pose2d getPose() {
        return this.getState().Pose;
    }

    public ChassisSpeeds getSpeeds() {
        return this.getState().Speeds;
    }

    public SwerveModuleState[] getModuleStates() {
        return this.getState().ModuleStates;
    }

    public double getRotationFromPigeon() {
        return this.getPigeon2().getYaw().getValueAsDouble() % 360;
    }

    public double getZeroOdoDegree() {
        return zeroOdoDegree;
    }

    // ======================= Manual driving =======================

    /**
     * Direct field-centric drive with raw velocities (m/s, m/s, rad/s).
     */
    public void driveFieldCentric(double vx, double vy, double omegaRadsPerSec) {
        this.setControl(m_manualDrive.withVelocityX(vx).withVelocityY(vy).withRotationalRate(omegaRadsPerSec));
    }

    /**
     * Field-centric drive straight from an Xbox controller:
     * left stick = translation, right stick = rotation.
     * Sticks get a signed power (expo) curve for finer control near center.
     */
    public void driveFieldCentric(ImprovedCommandXboxController controller) {
        double expo = 1.3; // >1 gives finer control around stick center
        driveFieldCentric(
            -signedPow(controller.getLeftY(), expo) * manual_MaxSpeed,
            -signedPow(controller.getLeftX(), expo) * manual_MaxSpeed,
            -controller.getRightX() * manual_MaxAngularRate
        );
    }

    /**
     * Robot-centric drive at raw velocities — handy for fine adjustment (D-pad nudges).
     */
    public void customMoveWithSpeed(double xSpeed, double ySpeed) {
        SwerveRequest.RobotCentric request = new SwerveRequest.RobotCentric();
        request = request.withVelocityX(xSpeed);
        request = request.withVelocityY(ySpeed);
        this.setControl(request);
    }

    /** Robot-centric rotation in place. */
    public void customRotate(double angularSpeed) {
        SwerveRequest.RobotCentric request = new SwerveRequest.RobotCentric();
        request = request.withRotationalRate(angularSpeed).withVelocityX(0).withVelocityY(0);
        this.setControl(request);
    }

    /** Zero all robot-centric velocities (pass true to actually do it). */
    public void customStopMoving(boolean doIt) {
        if (!doIt) {
            return;
        }
        SwerveRequest.RobotCentric request = new SwerveRequest.RobotCentric();
        request = request.withVelocityX(0).withVelocityY(0).withRotationalRate(0);
        this.setControl(request);
    }

    // ======================= Pose checks =======================

    public boolean isAtTranslation(Translation2d t) {
        return getPose().getTranslation().minus(t).getNorm() <= 1;
    }

    public boolean isAtRotation(Rotation2d r) {
        return Math.abs(getPose().getRotation().minus(r).getRadians()) <= 0.1;
    }

    public boolean isAtPose(Pose2d pose) {
        return isAtTranslation(pose.getTranslation()) && isAtRotation(pose.getRotation());
    }

    // ======================= Move to pose (PID) =======================

    /**
     * Computes chassis speeds that drive toward the given pose using the PID controllers.
     * Caution: this resets the stored target pose.
     */
    public ChassisSpeeds getAutoMoveToPoseSpeeds(Pose2d pose) {
        m_targetPose2d = pose;
        Pose2d currentPose = getPose();
        m_rotationController.setSetpoint(pose.getRotation().getRadians());
        m_translationXController.setSetpoint(pose.getX());
        m_translationYController.setSetpoint(pose.getY());

        double targetAngularVelocity = m_rotationController.calculate(currentPose.getRotation().getRadians());
        double targetTranslationXVelocity = m_translationXController.calculate(currentPose.getX());
        double targetTranslationYVelocity = m_translationYController.calculate(currentPose.getY());

        return optimizeMoveToSpeeds(
            new ChassisSpeeds(targetTranslationXVelocity, targetTranslationYVelocity, targetAngularVelocity)
        );
    }

    /** Same as {@link #getAutoMoveToPoseSpeeds(Pose2d)} with a translation speed clamp override. */
    public ChassisSpeeds getAutoMoveToPoseSpeeds(Pose2d pose, double maxMoveToSpeed) {
        m_targetPose2d = pose;
        Pose2d currentPose = getPose();
        m_rotationController.setSetpoint(pose.getRotation().getRadians());
        m_translationXController.setSetpoint(pose.getX());
        m_translationYController.setSetpoint(pose.getY());

        double targetAngularVelocity = m_rotationController.calculate(currentPose.getRotation().getRadians());
        double targetTranslationXVelocity = m_translationXController.calculate(currentPose.getX());
        double targetTranslationYVelocity = m_translationYController.calculate(currentPose.getY());

        return optimizeMoveToSpeeds(
            new ChassisSpeeds(targetTranslationXVelocity, targetTranslationYVelocity, targetAngularVelocity), maxMoveToSpeed
        );
    }

    public Command autoMoveToPoseCommand(Pose2d pose) {
        return this.applyRequest(() -> m_moveToPoseDrive.withSpeeds(getAutoMoveToPoseSpeeds(pose)));
    }

    public void autoMoveToPose(Pose2d pose) {
        this.setControl(m_moveToPoseDrive.withSpeeds(getAutoMoveToPoseSpeeds(pose)));
    }

    public void autoMoveToPose(Pose2d pose, double maxSpeed) {
        this.setControl(m_moveToPoseDrive.withSpeeds(getAutoMoveToPoseSpeeds(pose, maxSpeed)));
    }

    // ======================= Hybrid move to pose (driver-adjustable target) =======================

    /**
     * Like getAutoMoveToPoseSpeeds, but the driver's sticks offset the target pose:
     * left stick nudges translation, right stick rotates it. Good for aligning to a
     * scoring location while still letting the driver fine-tune.
     */
    public ChassisSpeeds getHybridMoveToPoseSpeeds(Pose2d pose, ImprovedCommandXboxController driverController, double translationAdjustmentRange, double rotationAdjustmentRangeDegs) {
        Pose2d currentPose = getPose();

        // Hybrid Control, read the lines carefully before you tune it
        boolean isBlueAlliance = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue;
        Transform2d adjustment = new Transform2d(
            driverController.getLeftY() * translationAdjustmentRange * (isBlueAlliance ? 1. : -1.),
            driverController.getLeftX() * translationAdjustmentRange * (isBlueAlliance ? 1. : -1.),
            Rotation2d.fromDegrees(-driverController.getRightX() * rotationAdjustmentRangeDegs)
        );
        pose = pose.plus(adjustment);
        m_targetPose2d = pose;

        m_rotationController.setSetpoint(pose.getRotation().getRadians());
        m_translationXController.setSetpoint(pose.getX());
        m_translationYController.setSetpoint(pose.getY());

        double targetAngularVelocity = m_rotationController.calculate(currentPose.getRotation().getRadians());
        double targetTranslationXVelocity = m_translationXController.calculate(currentPose.getX());
        double targetTranslationYVelocity = m_translationYController.calculate(currentPose.getY());

        if (isAtTranslation(pose.getTranslation())) {
            targetTranslationXVelocity = 0.;
            targetTranslationYVelocity = 0.;
        }
        if (isAtRotation(pose.getRotation())) {
            targetAngularVelocity = 0.;
        }

        return optimizeMoveToSpeeds(
            new ChassisSpeeds(targetTranslationXVelocity, targetTranslationYVelocity, targetAngularVelocity)
        );
    }

    public Command hybridMoveToPoseCommand(Supplier<Pose2d> pose, ImprovedCommandXboxController driverController, double translationAdjustmentRange, double rotationAdjustmentRangeDegs) {
        return this.applyRequest(() -> m_moveToPoseDrive.withSpeeds(getHybridMoveToPoseSpeeds(pose.get(), driverController, translationAdjustmentRange, rotationAdjustmentRangeDegs)));
    }

    public void hybridMoveToPose(Pose2d pose, ImprovedCommandXboxController driverController, double translationAdjustmentRange, double rotationAdjustmentRangeDegs) {
        this.setControl(m_moveToPoseDrive.withSpeeds(getHybridMoveToPoseSpeeds(pose, driverController, translationAdjustmentRange, rotationAdjustmentRangeDegs)));
    }

    // ======================= PathPlanner =======================

    /** Wires up PathPlanner's AutoBuilder. Call once at startup (RobotContainer constructor). */
    public void configureAutoBuilder() {
        try {
            var config = RobotConfig.fromGUISettings();
            AutoBuilder.configure(
                () -> getState().Pose,   // Supplier of current robot pose
                this::resetPose,         // Consumer for seeding pose against auto
                () -> getState().Speeds, // Supplier of current robot speeds
                // Consumer of ChassisSpeeds and feedforwards to drive the robot
                (speeds, feedforwards) -> setControl(
                    m_autoDrive.withSpeeds(ChassisSpeeds.discretize(speeds, 0.020))
                        .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                        .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())
                ),
                new PPHolonomicDriveController(
                    // PID constants for translation
                    new PIDConstants(15.0, 0.0, 0.0),
                    // PID constants for rotation
                    new PIDConstants(10.0, 0.0, 0.0)
                ),
                config,
                // Assume the path needs to be flipped for Red vs Blue, this is normally the case
                () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
                this // Subsystem for requirements
            );
        } catch (Exception ex) {
            DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder", ex.getStackTrace());
        }
    }

    /** Pathfind to a pose using PathPlanner (auto-flips for alliance). Returns the following command. */
    public Command autoMoveToPoseCommand_PP(Pose2d pose) {
        PathConstraints constraints = new PathConstraints(
            AutoConstants.maxMoveToSpeed,           // 3.0 m/s
            3.0,                                    // 4.5 m/s^2
            AutoConstants.maxMoveToAngularVelocity, // 200 deg/s
            Units.degreesToRadians(720.0)           // 720 deg/s^2
        );

        return AutoBuilder.pathfindToPoseFlipped(
            pose,
            constraints,
            0.0 // Goal end velocity
        );
    }

    // ======================= Heading / odometry zeroing =======================

    /**
     * Zeros the gyro heading so "forward" points at the given field angle,
     * and records the Pigeon/odometry offset used by the Limelight pipeline.
     */
    public void resetHeadingForOdo(double angle) {
        resetRotation(new Rotation2d(Units.degreesToRadians(angle)));
        zeroOdoDegree = getRotationFromPigeon() - angle;
    }

    /** Re-defines "forward" as the current facing for field-centric driving. */
    public void setFieldCentric() {
        setOperatorPerspectiveForward(getPose().getRotation());
    }

    /** Clears the rotation PID's accumulated error — call when switching control modes. */
    public void resetRotationController() {
        m_rotationController.reset();
    }

    // ======================= Internals =======================

    private double signedPow(double a, double b) {
        return Math.signum(a) * Math.pow(Math.abs(a), b);
    }

    /** Applies deadbands and clamps from AutoConstants to raw move-to-pose output. */
    private ChassisSpeeds optimizeMoveToSpeeds(ChassisSpeeds speeds) {
        ChassisSpeeds optimizedSpd = speeds;

        // If too small, apply deadband;
        optimizedSpd.vxMetersPerSecond = MathUtil.applyDeadband(optimizedSpd.vxMetersPerSecond, AutoConstants.moveToVelocityDeadband, 1e6);
        optimizedSpd.vyMetersPerSecond = MathUtil.applyDeadband(optimizedSpd.vyMetersPerSecond, AutoConstants.moveToVelocityDeadband, 1e6);
        optimizedSpd.omegaRadiansPerSecond = MathUtil.applyDeadband(optimizedSpd.omegaRadiansPerSecond, AutoConstants.moveToAnguVeloDeadband, 1e6);

        // if too big, apply limit;
        if (Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond) > AutoConstants.maxMoveToSpeed) {
            optimizedSpd = optimizedSpd.times(AutoConstants.maxMoveToSpeed / Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond));
        }
        if (Math.abs(speeds.omegaRadiansPerSecond) > AutoConstants.maxMoveToAngularVelocity) {
            optimizedSpd.omegaRadiansPerSecond = Math.copySign(AutoConstants.maxMoveToAngularVelocity, speeds.omegaRadiansPerSecond);
        }
        return optimizedSpd;
    }

    /** Variant of {@link #optimizeMoveToSpeeds(ChassisSpeeds)} with a per-call translation clamp. */
    private ChassisSpeeds optimizeMoveToSpeeds(ChassisSpeeds speeds, double maxMoveToSpeed) {
        ChassisSpeeds optimizedSpd = speeds;
        // If too small, apply deadband;
        optimizedSpd.vxMetersPerSecond = MathUtil.applyDeadband(optimizedSpd.vxMetersPerSecond, AutoConstants.moveToVelocityDeadband, 1e6);
        optimizedSpd.vyMetersPerSecond = MathUtil.applyDeadband(optimizedSpd.vyMetersPerSecond, AutoConstants.moveToVelocityDeadband, 1e6);
        optimizedSpd.omegaRadiansPerSecond = MathUtil.applyDeadband(optimizedSpd.omegaRadiansPerSecond, AutoConstants.moveToAnguVeloDeadband, 1e6);

        // if too big, apply limit;
        if (Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond) > maxMoveToSpeed) {
            optimizedSpd = optimizedSpd.times(maxMoveToSpeed / Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond));
        }
        if (Math.abs(speeds.omegaRadiansPerSecond) > AutoConstants.maxMoveToAngularVelocity) {
            optimizedSpd.omegaRadiansPerSecond = Math.copySign(AutoConstants.maxMoveToAngularVelocity, speeds.omegaRadiansPerSecond);
        }
        return optimizedSpd;
    }
}
