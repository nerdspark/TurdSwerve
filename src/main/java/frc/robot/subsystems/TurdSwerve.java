// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

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
import frc.robot.constants.Constants.RobotMap;
import frc.robot.constants.Constants.RobotConfig;


/**REV Turdswerve implementation */
public class TurdSwerve extends SubsystemBase {
    private final Pigeon2 gyro;

    private final TurdPod leftPod;
    private final TurdPod rightPod;
    
    private final SwerveDriveOdometry odometer;

    private ShuffleboardTab tab = Shuffleboard.getTab("PID");
    private GenericEntry azimuthP;
    private GenericEntry azimuthI;
    private GenericEntry azimuthD;
    private GenericEntry azimuthkS;
    private GenericEntry ADMult;

    private PIDController gyroPID;
    public double targetAngle = 0;
    private double odoAngleOffset = Math.PI * 0.0;

    private Rotation2d gyroResetAngle = new Rotation2d();
    
    private SwerveDriveKinematics drivetrainKinematics;
    private final double robotMaxSpeed;
    
    private final Field2d field2d = new Field2d();

    public TurdSwerve() {      
        
        this.gyroPID = RobotConfig.gyroPID;
        this.drivetrainKinematics = RobotConfig.drivetrainKinematics;
        this.robotMaxSpeed = RobotConfig.robotMaxSpeed;
        gyroPID.enableContinuousInput(0.0, 2*Math.PI);
        
        gyro = new Pigeon2(RobotMap.pigeonID);
        gyroPID.enableContinuousInput(0.0, 2*Math.PI);
        
        azimuthP = tab.add("azimuth P", RobotConfig.azimuthkP).getEntry();
        azimuthI = tab.add("azimuth I", RobotConfig.azimuthkI).getEntry();
        azimuthD = tab.add("azimuth D", RobotConfig.azimuthkD).getEntry();
        // change wildcard gain dependent on pod type
        azimuthkS = tab.add("azimuth kS", RobotConfig.azimuthkS).getEntry();
        ADMult = tab.add("Drive Speed Multiplier", RobotConfig.azimuthDriveSpeedMultiplier).getEntry();
        
        leftPod = new TurdPod(RobotMap.CAN_LeftAbsoluteEncoderID, RobotMap.leftAzimuthID, RobotMap.leftDriveID, RobotConfig.devastatorLeftOffset, RobotMap.leftAzimuthInvert, 
            RobotConfig.azimuthAmpLimit, RobotConfig.azimuthRadiansPerMotorRotation, RobotConfig.azimuthBrake, RobotConfig.azimuthMotorRampRate, RobotConfig.azimuthkP, 
            RobotConfig.azimuthkI, RobotConfig.azimuthkD, RobotConfig.azimuthkS, RobotConfig.azimuthMaxOutput, RobotConfig.azimuthDriveSpeedMultiplier, RobotMap.leftDriveInvert, 
            RobotConfig.driveAmpLimit, RobotConfig.driveBrake, RobotConfig.driveMotorRampRate);
        rightPod = new TurdPod(RobotMap.CAN_RightAbsoluteEncoderID, RobotMap.rightAzimuthID, RobotMap.rightDriveID, RobotConfig.devastatorRightOffset, RobotMap.rightAzimuthInvert, 
            RobotConfig.azimuthAmpLimit, RobotConfig.azimuthRadiansPerMotorRotation, RobotConfig.azimuthBrake, RobotConfig.azimuthMotorRampRate, RobotConfig.azimuthkP, 
            RobotConfig.azimuthkI, RobotConfig.azimuthkD, RobotConfig.azimuthkS, RobotConfig.azimuthMaxOutput, RobotConfig.azimuthDriveSpeedMultiplier, RobotMap.rightDriveInvert, 
            RobotConfig.driveAmpLimit, RobotConfig.driveBrake, RobotConfig.driveMotorRampRate);

        SwerveModulePosition positions[] = {rightPod.getPodPosition(), leftPod.getPodPosition()};

        odometer = new SwerveDriveOdometry(drivetrainKinematics, new Rotation2d(0), positions);

        AutoBuilder.configureHolonomic(
            this::getPose,
            this::resetOdometry,
            () -> drivetrainKinematics.toChassisSpeeds(getModuleStates()),
            this::setRobotSpeeds,
            new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                    new PIDConstants(0.5, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(0.5, 0.0, 0.0), // Rotation PID constants
                    RobotConfig.robotMaxSpeed, // Max module speed, in m/s
                    RobotConfig.driveBaseRadius, // Drive base radius in meters. Distance from robot center to furthest module.
                    new ReplanningConfig(false, false) // Default path replanning config. See the API for the options here
            ),
                () ->
                    DriverStation.getAlliance().isPresent()
                        && DriverStation.getAlliance().get() == Alliance.Red,
            this);


        // gyro.configAllSettings(new Pigeon2Configuration());
    }

    public void setAmpLimit(int ampLimit) {
        leftPod.setAmpLimit(ampLimit);
        rightPod.setAmpLimit(ampLimit);
    }

    public void resetOdometry(Pose2d pose) {
        odoAngleOffset = DriverStation.getAlliance().get() == Alliance.Red ? Math.PI * 0.5 : Math.PI * 1.5;
        odometer.resetPosition(new Rotation2d(odoAngleOffset), getModulePositions(), pose);
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = {leftPod.getPodPosition(), rightPod.getPodPosition()};
        return positions;
    }

    public void resetPods() {
        resetGyro();
        leftPod.resetPod();
        rightPod.resetPod();
        
        resetOdometry(new Pose2d(new Translation2d(8.0, 4.2), new Rotation2d()));
    }

    public Pose2d getPose() {
        return odometer.getPoseMeters();
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[2];
        // for (int i = 0; i < pods.length; i++) {
        //   states[i] = pods[i].getState();
        // }
        states[0] = leftPod.getState();
        states[1] = rightPod.getState();
        return states;
    }

    public void resetZero() {
        leftPod.resetZero();
        rightPod.resetZero();
    }

    public void revertZero() {
        leftPod.revertZero();
        rightPod.revertZero();
    }

    public void stop() {
        leftPod.stop();
        rightPod.stop();
    }

    public Rotation2d getGyro() {
        return new Rotation2d(gyro.getAngle()*Math.PI/180).minus(gyroResetAngle);
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

        leftPod.setPodState(states[0]);
        rightPod.setPodState(states[1]);
    }

    @Override
    public void periodic() {
        odometer.update(getGyro(), getModulePositions());
        SmartDashboard.putNumber("pigeon", getGyro().getDegrees());
        field2d.setRobotPose(odometer.getPoseMeters().transformBy(new Transform2d(new Translation2d(), new Rotation2d(odoAngleOffset + Math.PI))));

        //uncomment these lines for azimuth tuning
        // leftPod.setPID(azimuthkS.getDouble(SkywarpConfig.azimuthkS), azimuthP.getDouble(SkywarpConfig.azimuthkP), azimuthI.getDouble(SkywarpConfig.azimuthkI), azimuthD.getDouble(SkywarpConfig.azimuthkD), 1, ADMult.getDouble(SkywarpConfig.azimuthMaxOutput));
        // rightPod.setPID(azimuthkS.getDouble(SkywarpConfig.azimuthkS), azimuthP.getDouble(SkywarpConfig.azimuthkP), azimuthI.getDouble(SkywarpConfig.azimuthkI), azimuthD.getDouble(SkywarpConfig.azimuthkD), 1, ADMult.getDouble(SkywarpConfig.azimuthMaxOutput));
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
}
