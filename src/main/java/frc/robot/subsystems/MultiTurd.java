// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.hardware.pods.CTREPod;
import frc.robot.subsystems.hardware.pods.REVPod;
import frc.robot.subsystems.hardware.pods.TurdConfig;
import frc.robot.subsystems.hardware.pods.TurdConfig.PodType;
import frc.robot.subsystems.hardware.pods.TurdPod;
import java.util.function.BiConsumer;
import java.util.function.Consumer;


/**REV Turdswerve implementation */
public class MultiTurd extends SubsystemBase {
    private final Pigeon2 gyro;
    // private final TurdPod leftPod = new REVPod(RobotMap.leftAzimuthID, RobotMap.leftDriveID, RobotMap.leftAbsoluteEncoderID, RobotMap.leftAzimuthInvert, RobotMap.leftDriveInvert, RobotMap.leftAbsoluteEncoderOffset);
    // private final TurdPod rightPod = new REVPod(RobotMap.rightAzimuthID, RobotMap.rightDriveID, RobotMap.rightAbsoluteEncoderID, RobotMap.rightAzimuthInvert, RobotMap.rightDriveInvert, RobotMap.rightAbsoluteEncoderOffset);
    
    private final TurdPod[] pods;
    public final TurdConfig TemplateConf;
    
    private final SwerveDriveOdometry odometer;

    private ShuffleboardTab tab = Shuffleboard.getTab("PID");
    private GenericEntry azimuthP;
    private GenericEntry azimuthI;
    private GenericEntry azimuthD;
    private GenericEntry azimuthWildcard;
    private GenericEntry ADMult;

    private PIDController gyroPID;
    public double targetAngle = 0;
    private double odoAngleOffset = Math.PI * 0.0;

    private Rotation2d gyroResetAngle = new Rotation2d();
    
    private SwerveDriveKinematics drivetrainKinematics;
    private final double robotMaxSpeed;
    
    private final Field2d field2d = new Field2d();

    public MultiTurd(PIDController gyroPID, int pigeonID, SwerveDriveKinematics drivetrainKinematics, double robotMaxSpeed, TurdConfig templateConf, TurdConfig[] configs) {
        gyroPID.enableContinuousInput(0.0, 2*Math.PI);
        
        this.gyroPID = gyroPID;
        this.drivetrainKinematics = drivetrainKinematics;
        this.robotMaxSpeed = robotMaxSpeed;
        this.TemplateConf = templateConf;
        
        PodType podType = templateConf.podType;
        if(podType == PodType.undef) throw new IllegalArgumentException("Pod type must be defined in template config");
        
        pods = new TurdPod[configs.length];
        gyro = new Pigeon2(pigeonID);
        
        azimuthP = tab.add("azimuth P", templateConf.kP).getEntry();
        azimuthI = tab.add("azimuth I", templateConf.kI).getEntry();
        azimuthD = tab.add("azimuth D", templateConf.kD).getEntry();
        // change wildcard gain dependent on pod type
        azimuthWildcard = tab.add("azimuth " + (podType == PodType.REV ? "Iz" : "kF"), templateConf.wildcard).getEntry();
        ADMult = tab.add("Drive Speed Multiplier", templateConf.driveSpeedMult).getEntry();
        
        SwerveModulePosition positions[] = new SwerveModulePosition[configs.length];

        for(int i = 0; i < configs.length; i++) {
            TurdConfig config = configs[i];
            pods[i] = podType == PodType.REV ? new REVPod(config) : new CTREPod(config);

            positions[i] = pods[i].getPodPosition();
        }

        odometer = new SwerveDriveOdometry(drivetrainKinematics, new Rotation2d(0), positions);


        // gyro.configAllSettings(new Pigeon2Configuration());
    }

    public void setAmpLimit(int ampLimit) {
        forEachPod((TurdPod pod) -> pod.setAmpLimit(ampLimit));
    }

    // public void setDriveSpeedtoPower(double driveSpeedToPower) {
    //     leftPod.setDriveSpeedtoPower(driveSpeedToPower);
    //     rightPod.setDriveSpeedtoPower(driveSpeedToPower);
    // }

    public void resetOdometry(Pose2d pose) {
        odoAngleOffset = DriverStation.getAlliance().get() == Alliance.Red ? Math.PI * 0.5 : Math.PI * 1.5;
        odometer.resetPosition(new Rotation2d(odoAngleOffset), getModulePositions(), pose);
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[pods.length];
        forEachPodIndex((TurdPod pod, Integer index) -> positions[index] = pod.getPodPosition());
        return positions;
    }

    public void resetPods() {
        resetGyro();
        forEachPod(TurdPod::resetPod);
        
        resetOdometry(new Pose2d(new Translation2d(8.0, 4.2), new Rotation2d()));
    }

    public void resetZero() {
        forEachPod(TurdPod::resetZero);
    }

    public void revertZero() {
        forEachPod(TurdPod::revertZero);
    }

    public void stop() {
        forEachPod(TurdPod::stop);
    }

    public Rotation2d getGyro() {
        return new Rotation2d(-gyro.getAngle()*Math.PI/180).minus(gyroResetAngle);
    }

    public void resetGyro() {
        gyroResetAngle = getGyro().plus(gyroResetAngle);
        targetAngle = 0;
    }
    
    public void setRobotSpeeds(ChassisSpeeds chassisSpeeds) {
        boolean manualTurn = true; //Math.abs(chassisSpeeds.omegaRadiansPerSecond) > 0.1;

        chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond,
        manualTurn ? chassisSpeeds.omegaRadiansPerSecond * 3.0 : //TODO: magic number, please remove
        gyroPID.calculate(getGyro().getRadians(), targetAngle), getGyro());

        SwerveModuleState[] states = drivetrainKinematics.toSwerveModuleStates(chassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, robotMaxSpeed);
        
        if (manualTurn) {
            targetAngle = getGyro().getRadians() + (chassisSpeeds.omegaRadiansPerSecond / 2.0); //TODO: magic number
        }

        forEachPodIndex((TurdPod pod, Integer index) -> pod.setPodState(states[index]));
    }

    @Override
    public void periodic() {
        odometer.update(getGyro(), getModulePositions());
        SmartDashboard.putNumber("pigeon", getGyro().getDegrees());
        field2d.setRobotPose(odometer.getPoseMeters().transformBy(new Transform2d(new Translation2d(), new Rotation2d(odoAngleOffset + Math.PI))));

        //uncomment this line for azimuth tuning
        // forEachPod((TurdPod pod) -> pod.setPID(azimuthWildcard.getDouble(TemplateConf.wildcard), azimuthP.getDouble(TemplateConf.kP), azimuthI.getDouble(TemplateConf.kI), azimuthD.getDouble(TemplateConf.kD), 1, ADMult.getDouble(TemplateConf.maxOut)));
    }
    
    private String getFomattedPose() {
        var pose = odometer.getPoseMeters();
        return String.format(
                        "(%.3f, %.3f) %.2f degrees",
                        pose.getX(), pose.getY(), pose.getRotation().plus(new Rotation2d(odoAngleOffset)).getDegrees());
    }
    
    public void addDashboardWidgets(ShuffleboardTab tab) {
        tab.add("Field", field2d).withPosition(0, 0).withSize(6, 4);
        tab.addString("Pose", this::getFomattedPose).withPosition(6, 2).withSize(2, 1);
    }

    private void forEachPod(Consumer<TurdPod> action) {
        for(TurdPod pod : pods) {
            action.accept(pod);
        }
    }

    private void forEachPodIndex(BiConsumer<TurdPod, Integer> action) {
        for(int i = 0; i < pods.length; i++) {
            action.accept(pods[i], i);
        }
    }
}
