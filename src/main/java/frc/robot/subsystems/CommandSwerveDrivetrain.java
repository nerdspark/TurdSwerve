package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.PPLibTelemetry;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.OdometryConstants;
import frc.robot.generated.TunerConstants;
// import frc.robot.utils.MathFunctions;
import frc.robot.utils.MathFunctions2;
import frc.robot.utils.NerdOdometryRunnable;
// import frc.robot.utils.NerdOdometrySubsystem;
import frc.robot.utils.PointDW;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements
 * subsystem so it can be used in command-based projects easily.
 */
public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;
    private double odoTimeOld = 0.0;
    private double robotAngleOld = 0.0;
    private double robotOpticalEncoderStartX = 2.0;
    private double robotOpticalEncoderStartY = 7.0;
    private double xEncoderDistXOld = 2.0;
    private double yEncoderDistYOld = 7.0;

    private Encoder encoderXdw = new Encoder (8, 9, false, Encoder.EncodingType.k1X);
    // encoderXdw.setSamplesToAverage(5);
    // encoderXdw.setDistancePerPulse(1.0 / 360.0 * 2.0 * Math.PI * 1.75 / 2);
    // encoderXdw.setMinRate(1.0);
    private Encoder encoderYdw = new Encoder (2, 3, false, Encoder.EncodingType.k1X);
    // encoderYdw.setSamplesToAverage(5);
    // encoderYdw.setDistancePerPulse(1.0 / 360.0 * 2.0 * Math.PI * 1.75 / 2);
    // encoderYdw.setMinRate(1.0);
    private static Timer deadWheelElapsedTime = new Timer();

    private final Field2d field2d = new Field2d();

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private final Rotation2d BlueAlliancePerspectiveRotation = Rotation2d.fromDegrees(0);
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private final Rotation2d RedAlliancePerspectiveRotation = Rotation2d.fromDegrees(180);
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean hasAppliedOperatorPerspective = false;

    /*Nerd Odometry Subsystem uses a deadwheel odometry calculation and enables other control system features, like crash recovery*/
    // private NerdOdometrySubsystem nerdOdometrySubsystem;

    // private DeadWheelSubsystem deadWheelSubsystem;

    private final SwerveRequest.ApplyChassisSpeeds AutoRequest = new SwerveRequest.ApplyChassisSpeeds();

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency, SwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);
        configurePathPlanner();
        configureDeadWheel();
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }
    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
        configurePathPlanner();
        configureDeadWheel();
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    private void configurePathPlanner() {
        double driveBaseRadius = Units.inchesToMeters(8);
        for (var moduleLocation : m_moduleLocations) {
            driveBaseRadius = Math.max(driveBaseRadius, moduleLocation.getNorm());
        }

        AutoBuilder.configureHolonomic(
            ()->this.getState().Pose, // Supplier of current robot pose
            // this::NerdOdometrySubsystem.deadWheelEstimator.returnDeadWheelPose(),
            // this::NerdOdometrySubsystem.updateNerdOdo.deadWheelPose();
            this::seedFieldRelative,  // Consumer for seeding pose against auto
            this::getCurrentRobotChassisSpeeds,
            (speeds)->this.setControl(AutoRequest.withSpeeds(speeds).withDriveRequestType(DriveRequestType.Velocity)), // Consumer of ChassisSpeeds to drive the robot
            new HolonomicPathFollowerConfig(new PIDConstants(10, 0, 0),
                                            new PIDConstants(10, 0, 0),
                                            TunerConstants.kSpeedAt12VoltsMps,
                                            driveBaseRadius,
                                            new ReplanningConfig(false, false)),
            () -> DriverStation.getAlliance().orElse(Alliance.Blue)==Alliance.Red, // Assume the path needs to be flipped for Red vs Blue, this is normally the case
            this); // Subsystem for requirements
    }

    private void configureDeadWheel(){
        encoderXdw.setSamplesToAverage(5);
        encoderXdw.setDistancePerPulse(1.0 / 360.0 * 2.0 * Math.PI * (OdometryConstants.DEADWHEEL_WHEEL_DIAMETER / 2));
        encoderXdw.setMinRate(1.0);
        encoderYdw.setSamplesToAverage(5);
        encoderYdw.setDistancePerPulse(1.0 / 360.0 * 2.0 * Math.PI * (OdometryConstants.DEADWHEEL_WHEEL_DIAMETER / 2));
        encoderYdw.setMinRate(1.0);
    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    public ChassisSpeeds getCurrentRobotChassisSpeeds() {
        return m_kinematics.toChassisSpeeds(getState().ModuleStates);
    }

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

    // /**** Vision Pose ****/
    public void addDashboardWidgets(ShuffleboardTab tab) {
        tab.add("Field", field2d).withPosition(0, 0).withSize(6, 4);
        tab.addString("Pose", this::getFomattedPose).withPosition(6, 2).withSize(2, 1);
    }

    private String getFomattedPose() {
        var pose = this.getState().Pose;
        return String.format(
                "(%.3f, %.3f) %.2f degrees",
                pose.getX(), pose.getY(), pose.getRotation().getDegrees());
    }


    // // /**** Dead Wheel Odometry Pose ****/
    // public void addDashboardWidgetsOdo(ShuffleboardTab tab) {
    //     tab.add("FieldOdo", field2d).withPosition(0, 0).withSize(6, 4);
    //     tab.addString("PoseOdo", this::getFomattedPoseOdo).withPosition(6, 2).withSize(2, 1);
    // }

    // private String getFomattedPoseOdo() {
    //     // var poseOdo = this.getState().Pose;
    //     // var poseOdo = nerdOdometrySubsystem.getCurrentPose();
    //     var poseOdo = nerdOdometrySubsystem.getCurrentPoseOdo();
    //     return String.format(
    //             "(%.3f, %.3f) %.2f degrees",
    //             poseOdo.getX(), poseOdo.getY(), poseOdo.getRotation().getDegrees());
    // }



    public Pose2d getCurrentPose() {
        return this.getState().Pose;
    }

    @Override
    public void periodic() {
        /* Periodically try to apply the operator perspective */
        /* If we haven't applied the operator perspective before, then we should apply it regardless of DS state */
        /* This allows us to correct the perspective in case the robot code restarts mid-match */
        /* Otherwise, only check and apply the operator perspective if the DS is disabled */
        /* This ensures driving behavior doesn't change until an explicit disable event occurs during testing*/
        if (!hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent((allianceColor) -> {
                this.setOperatorPerspectiveForward(
                        allianceColor == Alliance.Red ? RedAlliancePerspectiveRotation
                                : BlueAlliancePerspectiveRotation);
                hasAppliedOperatorPerspective = true;
            });
        }

        for (int i = 0; i < 2; i++) {
            deadWheelElapsedTime.reset();
            deadWheelElapsedTime.start();
        }

        var dashboardPose = this.getState().Pose;
        field2d.setRobotPose(dashboardPose);

        // ArrayList<PointDW> deadWheelODO = MathFunctions.globalCoordinatePositionUpdateNERD(encoderXdw.getDistance(), encoderYdw.getDistance(), getState().Pose.getRotation().getDegrees());
        double deadWheelOdo [] = MathFunctions2.globalCoordinatePositionUpdateNERD(encoderXdw.getDistance(), xEncoderDistXOld, encoderXdw.getRate(), 
            encoderYdw.getDistance(), yEncoderDistYOld, encoderYdw.getRate(), getState().Pose.getRotation().getRadians(), robotAngleOld, deadWheelElapsedTime.get(),
            odoTimeOld, robotOpticalEncoderStartX, robotOpticalEncoderStartY);

        SmartDashboard.putNumber("Loop Time", deadWheelElapsedTime.get() - odoTimeOld);

        odoTimeOld = deadWheelElapsedTime.get();
        robotAngleOld = getState().Pose.getRotation().getRadians();

        SmartDashboard.putNumber("X Encoder", encoderXdw.getDistance());
        SmartDashboard.putNumber("Y Encoder", encoderYdw.getDistance());

        SmartDashboard.putNumber("X Opt Enc Position", deadWheelOdo[0]);
        SmartDashboard.putNumber("Y Opt Enc Position", deadWheelOdo[1]);

        SmartDashboard.putNumber("X Kraken Position", getState().Pose.getX());
        SmartDashboard.putNumber("Y Kraken Position", getState().Pose.getY());

        SmartDashboard.putNumber("X Rate", encoderXdw.getRate());
        SmartDashboard.putNumber("Y Rate", encoderYdw.getRate());

        SmartDashboard.putNumber("Robot Angle", getState().Pose.getRotation().getRadians());
        SmartDashboard.putNumber("Robot Angle Old", robotAngleOld);

        SmartDashboard.putNumber("Start Pose X", robotOpticalEncoderStartX);
        SmartDashboard.putNumber("Start Pose Y", robotOpticalEncoderStartY);

        robotOpticalEncoderStartX = deadWheelOdo[0];
        robotOpticalEncoderStartY = deadWheelOdo[1];

        xEncoderDistXOld = encoderXdw.getDistance();
        yEncoderDistYOld = encoderYdw.getDistance();

    }

    // public void setNerdOdometrySubsystem(NerdOdometrySubsystem nerdOdometry) {
    //     this.nerdOdometrySubsystem = nerdOdometry;
    // }

    // public void setDeadWheelSubsystem(DeadWheelSubsystem deadWheelOdometry) {
    //     this.deadWheelSubsystem = deadWheelOdometry;
    // }

}
