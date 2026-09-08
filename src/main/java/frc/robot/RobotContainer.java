// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;

import frc.robot.Library.ImprovedCommandXboxController;
import frc.robot.Library.Scope;
import frc.robot.commands.fineTuneDrivetrainCmd;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class RobotContainer {
    private final double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // top speed at 12 V
    private final Telemetry logger = new Telemetry(MaxSpeed);
    private final CommandScheduler cs = CommandScheduler.getInstance();

    // Subsystems: add yours below as you build them out.
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    // Three controllers: driver, operator, test. Rename/repurpose as needed.
    public static final ImprovedCommandXboxController m_driverController = new ImprovedCommandXboxController(0);
    public static final ImprovedCommandXboxController m_driverController2 = new ImprovedCommandXboxController(1);
    public static final ImprovedCommandXboxController m_driverController3 = new ImprovedCommandXboxController(2);

    private final SendableChooser<Command> autoChooser;

    // Publishers used by update() to feed AdvantageScope (pose + module states).
    private final StructPublisher<Pose2d> robotPosPublisher = NetworkTableInstance.getDefault()
        .getStructTopic("MyPose", Pose2d.struct).publish();

    private final StructArrayPublisher<SwerveModuleState> swerveStatePublisher = NetworkTableInstance.getDefault()
        .getStructArrayTopic("/MyStates", SwerveModuleState.struct).publish();

    // Heading the robot should face at auto start, per alliance.
    // Blue alliance: 0 deg. Red alliance: 180 deg.
    private static final double HEADING_BLUE = 0;
    private static final double HEADING_RED = 180;

    private DriverStation.Alliance lastAlliance = null;
    private boolean lastEnabled = false;

    public RobotContainer() {
        // PathPlanner: reads robot config + autos from deploy/pathplanner
        drivetrain.configureAutoBuilder();
        autoChooser = AutoBuilder.buildAutoChooser(""); // no default auto — pick from dashboard

        SmartDashboard.putData("Auto Chooser", autoChooser);

        configureBindings();
    }

    private void configureBindings() {
        // Manual field-centric driving is the drivetrain's default command.
        drivetrain.setDefaultCommand(
            drivetrain.run(() -> drivetrain.driveFieldCentric(m_driverController))
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        // Face the correct alliance wall at startup so odometry/MT2 vision start sane.
        double headingAngle = HEADING_BLUE;
        if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
            headingAngle = HEADING_RED;
        }
        drivetrain.resetHeadingForOdo(headingAngle);

        configureDriver1Bindings();
        configureDriver2Bindings();

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    /*
     * Driver 1 (port 0):
     *   Left stick       - translate (field-centric)
     *   Right stick      - rotate
     *   Back             - re-define "forward" as current facing
     *   D-pad / triggers - fine adjustment (creep + rotate in place)
     */
    private void configureDriver1Bindings() {
        // reset the field-centric heading on back button press
        m_driverController.back().onTrue(drivetrain.runOnce(() -> drivetrain.setFieldCentric()));

        // Fine adjustment: D-pad nudges translation, triggers rotate in place.
        m_driverController.povUp().whileTrue(new fineTuneDrivetrainCmd(drivetrain, 0));
        m_driverController.povLeft().whileTrue(new fineTuneDrivetrainCmd(drivetrain, 1));
        m_driverController.povDown().whileTrue(new fineTuneDrivetrainCmd(drivetrain, 2));
        m_driverController.povRight().whileTrue(new fineTuneDrivetrainCmd(drivetrain, 3));

        m_driverController.leftTrigger().whileTrue(new fineTuneDrivetrainCmd(drivetrain, 4));
        m_driverController.rightTrigger().whileTrue(new fineTuneDrivetrainCmd(drivetrain, 5));
    }

    /*
     * Driver 2 (port 1): free — add operator controls here.
     */
    private void configureDriver2Bindings() {
    }

    private void configureDriver3Bindings() {
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    public void update() {
        Pose2d pos = drivetrain.getPose();

        swerveStatePublisher.set(drivetrain.getModuleStates());
        robotPosPublisher.set(pos);

        // Example Scope usage: one-line logging visible in AdvantageScope live AND in the log file.
        var speeds = drivetrain.getSpeeds();
        Scope.put("Drivetrain/Speed", Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond));
        Scope.put("Drivetrain/Omega", speeds.omegaRadiansPerSecond);
    }

    /**
     * Runs every loop even when disabled. Handles things that must track
     * DS state: alliance changes and Limelight IMU mode.
     */
    public void updateAlways() {
        var allianceOpt = DriverStation.getAlliance();
        if (allianceOpt.isPresent()) {
            var alliance = allianceOpt.get();
            if (alliance != lastAlliance) {
                lastAlliance = alliance;

                // Only reset heading while disabled to prevent mid-match glitches.
                if (DriverStation.isDisabled()) {
                    double headingAngle = (alliance == DriverStation.Alliance.Red) ? HEADING_RED : HEADING_BLUE;
                    drivetrain.resetHeadingForOdo(headingAngle);
                }
            }
        }

        // Dynamic Limelight IMU mode: use external IMU while enabled (lower latency),
        // internal while disabled so it doesn't fight the gyro at rest.
        boolean isEnabled = DriverStation.isEnabled();
        if (isEnabled != lastEnabled) {
            lastEnabled = isEnabled;
            int imuMode = isEnabled ? 4 : 1;

            LimelightHelpers.SetIMUMode(Constants.VisionConstants.LIMELIGHT_NAME, imuMode);
        }
    }

    public void telInit() {
    }

    public void autoInit() {
    }

    public void testInit() {
        configureDriver3Bindings();
    }

    public void onDisabled() {}
}
