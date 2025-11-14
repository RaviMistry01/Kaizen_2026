package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.pathfinding.Pathfinding;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.
 */
public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;
    LimelightHelpers.PoseEstimate mt2 = new PoseEstimate();
    boolean rejectUpdate;
    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean m_hasAppliedOperatorPerspective = false;

    /** Swerve request to apply during robot-centric path following */
    private final SwerveRequest.ApplyRobotSpeeds m_pathApplyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();

    /* Swerve requests to apply during SysId characterization */
    private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
    private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();

    private boolean autoRunning = false;
    private boolean visionEnabled = true;
    private double autoStartTime = 0;

    private final Field2d field = new Field2d();

    /*
     * SysId routine for characterizing translation. This is used to find PID gains
     * for the drive motors.
     */
    private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
            new SysIdRoutine.Config(
                    null, // Use default ramp rate (1 V/s)
                    Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
                    null, // Use default timeout (10 s)
                    // Log state with SignalLogger class
                    state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())),
            new SysIdRoutine.Mechanism(
                    output -> setControl(m_translationCharacterization.withVolts(output)),
                    null,
                    this));

    /*
     * SysId routine for characterizing steer. This is used to find PID gains for
     * the steer motors.
     */
    private final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine(
            new SysIdRoutine.Config(
                    null, // Use default ramp rate (1 V/s)
                    Volts.of(7), // Use dynamic voltage of 7 V
                    null, // Use default timeout (10 s)
                    // Log state with SignalLogger class
                    state -> SignalLogger.writeString("SysIdSteer_State", state.toString())),
            new SysIdRoutine.Mechanism(
                    volts -> setControl(m_steerCharacterization.withVolts(volts)),
                    null,
                    this));

    /*
     * SysId routine for characterizing rotation.
     * This is used to find PID gains for the FieldCentricFacingAngle
     * HeadingController.
     * See the documentation of SwerveRequest.SysIdSwerveRotation for info on
     * importing the log to SysId.
     */
    private final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(
            new SysIdRoutine.Config(
                    /* This is in radians per second², but SysId only supports "volts per second" */
                    Volts.of(Math.PI / 6).per(Second),
                    /* This is in radians per second, but SysId only supports "volts" */
                    Volts.of(Math.PI),
                    null, // Use default timeout (10 s)
                    // Log state with SignalLogger class
                    state -> SignalLogger.writeString("SysIdRotation_State", state.toString())),
            new SysIdRoutine.Mechanism(
                    output -> {
                        /* output is actually radians per second, but SysId only supports "volts" */
                        setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
                        /* also log the requested output for SysId */
                        SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
                    },
                    null,
                    this));

    /* The SysId routine to test */
    private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineTranslation;

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not
     * construct
     * the devices themselves. If they need the devices, they can access them
     * through
     * getters in the classes.
     *
     * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
     * @param modules             Constants for each specific module
     */
    public CommandSwerveDrivetrain(
            SwerveDrivetrainConstants drivetrainConstants,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        configureAutoBuilder();

        SmartDashboard.putData("FieldPath", field);

    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not
     * construct
     * the devices themselves. If they need the devices, they can access them
     * through
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
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, odometryUpdateFrequency, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        configureAutoBuilder();
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not
     * construct
     * the devices themselves. If they need the devices, they can access them
     * through
     * getters in the classes.
     *
     * @param drivetrainConstants       Drivetrain-wide constants for the swerve
     *                                  drive
     * @param odometryUpdateFrequency   The frequency to run the odometry loop. If
     *                                  unspecified or set to 0 Hz, this is 250 Hz
     *                                  on
     *                                  CAN FD, and 100 Hz on CAN 2.0.
     * @param odometryStandardDeviation The standard deviation for odometry
     *                                  calculation
     *                                  in the form [x, y, theta]ᵀ, with units in
     *                                  meters
     *                                  and radians
     * @param visionStandardDeviation   The standard deviation for vision
     *                                  calculation
     *                                  in the form [x, y, theta]ᵀ, with units in
     *                                  meters
     *                                  and radians
     * @param modules                   Constants for each specific module
     */
    public CommandSwerveDrivetrain(
            SwerveDrivetrainConstants drivetrainConstants,
            double odometryUpdateFrequency,
            Matrix<N3, N1> odometryStandardDeviation,
            Matrix<N3, N1> visionStandardDeviation,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation,
                modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        configureAutoBuilder();
    }

    private void configureAutoBuilder() {
        try {
            var config = RobotConfig.fromGUISettings();
            AutoBuilder.configure(
                    () -> getState().Pose, // Supplier of current robot pose
                    this::resetPose, // Consumer for seeding pose against auto
                    () -> getState().Speeds, // Supplier of current robot speeds
                    // Consumer of ChassisSpeeds and feedforwards to drive the robot
                    (speeds, feedforwards) -> setControl(
                            m_pathApplyRobotSpeeds.withSpeeds(speeds)
                                    .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                                    .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())),
                    new PPHolonomicDriveController(
                            // PID constants for translation
                            new PIDConstants(5, 0, 0),
                            // PID constants for rotation
                            new PIDConstants(5, 0, 0)),
                    config,
                    // Assume the path needs to be flipped for Red vs Blue, this is normally the
                    // case
                    () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue,
                    this // Subsystem for requirements
            );
        } catch (Exception ex) {
            DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder",
                    ex.getStackTrace());
        }
    }

    /**
     * Returns a command that applies the specified control request to this swerve
     * drivetrain.
     *
     * @param request Function returning the request to apply
     * @return Command to run
     */
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    /**
     * Runs the SysId Quasistatic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Quasistatic test
     * @return Command to run
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.quasistatic(direction);
    }

    /**
     * Runs the SysId Dynamic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Dynamic test
     * @return Command to run
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.dynamic(direction);
    }

    // public void setAutoRunning(boolean running) {
    // autoRunning = running;

    // if (running) {
    // // ✅ Disable vision AFTER initial pose is set
    // // Give MT2 ~0.3 seconds to update initial pose
    // autoStartTime = Utils.getCurrentTimeSeconds();
    // visionEnabled = true;
    // } else {
    // // ✅ Enable vision again after auto
    // visionEnabled = true;
    // }
    // }

    @Override
    public void periodic() {

        // ✅ Simulation skip
        // if (Utils.isSimulation()) return;

        // ✅ 1. Get MT2 pose
        mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-l");

        rejectUpdate = false;

        // // ✅ 2. If auto is running, disable vision AFTER pose is initialized
        // if (autoRunning) {

        // double timeSinceStart = Utils.getCurrentTimeSeconds() - autoStartTime;

        // if (timeSinceStart > 0.25) {
        // // ✅ Vision is DISABLED during auto path-following
        // visionEnabled = false;
        // }
        // }

        // // ✅ 3. Vision disabled? Then skip ALL tag updates
        // if (!visionEnabled) {
        // return; // ✅ Robot now runs purely on odometry
        // }

        // ✅ 4. Normal rejection rules (only used BEFORE auto, or in teleop)
        if (mt2 == null || mt2.tagCount == 0)
            rejectUpdate = true;

        if (Math.abs(getPigeon2().getAngularVelocityZDevice().getValueAsDouble()) > 720)
            rejectUpdate = true;

        // ✅ 5. Apply MT2 once vision is enabled
        if (!rejectUpdate) {
            // We trust X/Y but ignore theta
            setVisionMeasurementStdDevs(VecBuilder.fill(0.7, 0.7, 9999999));
            addVisionMeasurement(mt2.pose, mt2.timestampSeconds);
        }

        field.setRobotPose(getState().Pose);

        // ✅ 6. Dashboard
        SmartDashboard.putNumber("Vision X", (mt2 != null) ? mt2.pose.getX() : -99);
        SmartDashboard.putNumber("Vision Y", (mt2 != null) ? mt2.pose.getY() : -99);
        SmartDashboard.putNumber("Robot X", getState().Pose.getX());
        SmartDashboard.putNumber("Robot Y", getState().Pose.getY());
        SmartDashboard.putNumber("Heading", getState().Pose.getRotation().getDegrees());

        // ✅ Operator Perspective logic
        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                setOperatorPerspectiveForward(
                        allianceColor == Alliance.Red ? kRedAlliancePerspectiveRotation
                                : kBlueAlliancePerspectiveRotation);
                m_hasAppliedOperatorPerspective = true;
            });
        }
    }

    // @Override
    // public void periodic() {

    // // if (Utils.isSimulation()) {
    // // return;
    // // }

    // mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-l");
    // LimelightHelpers.SetRobotOrientation("limelight-l",getPigeon2().getYaw().getValueAsDouble(),0.0,0.0,0.0,0.0,0.0);
    // rejectUpdate = false;

    // if (mt2 == null || mt2.tagCount == 0)
    // rejectUpdate = true;

    // if (Math.abs(getPigeon2().getAngularVelocityZDevice().getValueAsDouble()) >
    // 720)
    // rejectUpdate = true;

    // if (!rejectUpdate) {
    // setVisionMeasurementStdDevs(VecBuilder.fill(0.7, 0.7, 9999999));
    // addVisionMeasurement(mt2.pose, mt2.timestampSeconds);
    // }

    // SmartDashboard.putNumber("Vision X", (mt2 != null) ? mt2.pose.getX() : -99);
    // SmartDashboard.putNumber("Vision Y", (mt2 != null) ? mt2.pose.getY() : -99);
    // SmartDashboard.putNumber("Robot X", getState().Pose.getX());
    // SmartDashboard.putNumber("Robot Y", getState().Pose.getY());

    // if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
    // DriverStation.getAlliance().ifPresent(allianceColor -> {
    // setOperatorPerspectiveForward(
    // allianceColor == Alliance.Red ? kRedAlliancePerspectiveRotation
    // : kBlueAlliancePerspectiveRotation);
    // m_hasAppliedOperatorPerspective = true;
    // });
    // }
    // }

    // @Override
    // public void periodic() {

    // // ✅ 1. If simulation → skip Limelight completely (no crashes)
    // if (Utils.isSimulation()) {

    // // Optional: show simulated pose
    // SmartDashboard.putNumber("Sim Robot X", getState().Pose.getX());
    // SmartDashboard.putNumber("Sim Robot Y", getState().Pose.getY());
    // SmartDashboard.putNumber("Sim Robot Heading",
    // getState().Pose.getRotation().getDegrees());

    // return; // ✅ Do NOT run Limelight code in sim
    // }

    // // ✅ 2. REAL ROBOT: MegaTag2 vision logic
    // mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-l");
    // rejectUpdate = false;

    // // Reject if null
    // if (mt2 == null || mt2.tagCount == 0)
    // rejectUpdate = true;

    // // Reject if spinning too fast
    // if (Math.abs(getPigeon2().getAngularVelocityZDevice().getValueAsDouble()) >
    // 720)
    // rejectUpdate = true;

    // // ✅ Send filtered vision pose to CTRE estimator
    // if (!rejectUpdate) {
    // setVisionMeasurementStdDevs(VecBuilder.fill(0.7, 0.7, 9999999));
    // addVisionMeasurement(mt2.pose, mt2.timestampSeconds);
    // }

    // // ✅ Dashboard output
    // SmartDashboard.putNumber("Vision X", (mt2 != null) ? mt2.pose.getX() : -99);
    // SmartDashboard.putNumber("Vision Y", (mt2 != null) ? mt2.pose.getY() : -99);
    // SmartDashboard.putNumber("Robot X", getState().Pose.getX());
    // SmartDashboard.putNumber("Robot Y", getState().Pose.getY());
    // SmartDashboard.putNumber("Robot Heading",
    // getPigeon2().getYaw().getValueAsDouble());

    // // ✅ Operator perspective (field-centric)
    // if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
    // DriverStation.getAlliance().ifPresent(allianceColor -> {
    // setOperatorPerspectiveForward(
    // allianceColor == Alliance.Red
    // ? kRedAlliancePerspectiveRotation
    // : kBlueAlliancePerspectiveRotation);
    // m_hasAppliedOperatorPerspective = true;
    // });
    // }
    // }

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

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the
     * odometry pose estimate
     * while still accounting for measurement noise.
     *
     * @param visionRobotPoseMeters The pose of the robot as measured by the vision
     *                              camera.
     * @param timestampSeconds      The timestamp of the vision measurement in
     *                              seconds.
     */
    @Override
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds));
    }

    public Command pathfindTo(double x, double y, double headingDeg) {
    Pose2d target = new Pose2d(x, y, Rotation2d.fromDegrees(headingDeg));

    PathConstraints constraints = new PathConstraints(
            2.0, // Max linear velocity (m/s)
            1.5, // Max linear acceleration (m/s^2)
            Math.toRadians(200), // Max angular velocity (rad/s)
            Math.toRadians(250)  // Max angular acceleration (rad/s^2)
    );

    // Create the actual auto command
    Command pathCommand = AutoBuilder.pathfindToPose(target, constraints, 0.0);

    // If you’re running in simulation, visualize the generated path
    if (Utils.isSimulation()) {
        PathPlannerPath currentPath = Pathfinding.getCurrentPath(
                constraints,
                new GoalEndState(0.0, Rotation2d.fromDegrees(headingDeg))
        );

        if (currentPath != null) {
            field.getObject("Generated Path").setPoses(currentPath.getPathPoses());
        }
    }

    return pathCommand;
}


    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the
     * odometry pose estimate
     * while still accounting for measurement noise.
     * <p>
     * Note that the vision measurement standard deviations passed into this method
     * will continue to apply to future measurements until a subsequent call to
     * {@link #setVisionMeasurementStdDevs(Matrix)} or this method.
     *
     * @param visionRobotPoseMeters    The pose of the robot as measured by the
     *                                 vision camera.
     * @param timestampSeconds         The timestamp of the vision measurement in
     *                                 seconds.
     * @param visionMeasurementStdDevs Standard deviations of the vision pose
     *                                 measurement
     *                                 in the form [x, y, theta]ᵀ, with units in
     *                                 meters and radians.
     */
    @Override
    public void addVisionMeasurement(
            Pose2d visionRobotPoseMeters,
            double timestampSeconds,
            Matrix<N3, N1> visionMeasurementStdDevs) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds),
                visionMeasurementStdDevs);
    }
}
