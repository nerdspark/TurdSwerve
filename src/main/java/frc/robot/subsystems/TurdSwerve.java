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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class TurdSwerve extends SubsystemBase {
  private final Pigeon2 gyro = new Pigeon2(Constants.pigeonID);
  private final TurdPod leftPod = new TurdPod(Constants.leftAzimuthID, Constants.leftDriveID, Constants.leftAbsoluteEncoderID, Constants.leftAzimuthInvert, Constants.leftDriveInvert, Constants.leftAbsoluteEncoderOffset);
  private final TurdPod rightPod = new TurdPod(Constants.rightAzimuthID, Constants.rightDriveID, Constants.rightAbsoluteEncoderID, Constants.rightAzimuthInvert, Constants.rightDriveInvert, Constants.rightAbsoluteEncoderOffset);
  private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(Constants.drivetrainKinematics,
          new Rotation2d(0), new SwerveModulePosition[] {
              leftPod.getPodPosition(),
              rightPod.getPodPosition()
          });

  private ShuffleboardTab tab = Shuffleboard.getTab("PID");
  private GenericEntry azimuthP = tab.add("azimuth P", Constants.azimuthkP).getEntry();
  private GenericEntry azimuthI = tab.add("azimuth I", Constants.azimuthkI).getEntry();
  private GenericEntry azimuthD = tab.add("azimuth D", Constants.azimuthkD).getEntry();
  private GenericEntry azimuthIzone = tab.add("azimuth IZone", Constants.azimuthkD).getEntry();
  private GenericEntry ADMult = tab.add("azimuth-drive speed multiplier", Constants.azimuthDriveSpeedMultiplier).getEntry();

  private PIDController GyroPID = new PIDController(Constants.gyroP, Constants.gyroI, Constants.gyroD);
  public double targetAngle = 0;
  private double odoAngleOffset = Math.PI * 0.5;

  private Rotation2d gyroResetAngle = new Rotation2d();
  
  
  private final Field2d field2d = new Field2d();

  public TurdSwerve() {
    GyroPID.enableContinuousInput(0.0, 2*Math.PI);
    // gyro.configAllSettings(new Pigeon2Configuration());
  }

  public void resetOdometry(Pose2d pose) {
    odoAngleOffset = DriverStation.getAlliance().get() == Alliance.Red ? Math.PI * 0.5 : Math.PI * 1.5;
    odometer.resetPosition(new Rotation2d(odoAngleOffset), new SwerveModulePosition[] {leftPod.getPodPosition(), rightPod.getPodPosition()}, pose);
  }

  public void resetPods() {
    resetGyro();
    leftPod.resetPod();
    rightPod.resetPod();
    leftPod.setPID(azimuthP.getDouble(Constants.azimuthkP), azimuthI.getDouble(Constants.azimuthkI), azimuthD.getDouble(Constants.azimuthkD), azimuthIzone.getDouble(Constants.azimuthkIz), Constants.azimuthMaxOutput, ADMult.getDouble(Constants.azimuthDriveSpeedMultiplier));
    rightPod.setPID(azimuthP.getDouble(Constants.azimuthkP), azimuthI.getDouble(Constants.azimuthkI), azimuthD.getDouble(Constants.azimuthkD), azimuthIzone.getDouble(Constants.azimuthkIz), Constants.azimuthMaxOutput, ADMult.getDouble(Constants.azimuthDriveSpeedMultiplier));
    resetOdometry(new Pose2d(new Translation2d(12.0, 4.2), new Rotation2d()));
  }

  public Rotation2d getGyro() {
    return new Rotation2d(-gyro.getAngle()*Math.PI/180).minus(gyroResetAngle);
  }

  public void resetGyro() {
    gyroResetAngle = getGyro().plus(gyroResetAngle);
    targetAngle = 0;
  }

  public void setLeftPod(SwerveModuleState state) {
    leftPod.setPodState(state);
  }

  public void setRobotSpeeds(ChassisSpeeds chassisSpeeds) {
    boolean manualTurn = Math.abs(chassisSpeeds.omegaRadiansPerSecond) > 0.1;
    chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond, manualTurn ? chassisSpeeds.omegaRadiansPerSecond * 3.0 : GyroPID.calculate(getGyro().getRadians(), targetAngle), getGyro());
    SwerveModuleState[] states = Constants.drivetrainKinematics.toSwerveModuleStates(chassisSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.podMaxSpeed);
    if (manualTurn) {
    targetAngle = getGyro().getRadians() + (chassisSpeeds.omegaRadiansPerSecond / 2.0);
    }

    leftPod.setPodState(states[0]);
    rightPod.setPodState(states[1]);
  }

  @Override
  public void periodic() {
    odometer.update(getGyro(), new SwerveModulePosition[] {leftPod.getPodPosition(), rightPod.getPodPosition()});
    SmartDashboard.putNumber("pigeon", getGyro().getDegrees());
    field2d.setRobotPose(odometer.getPoseMeters().transformBy(new Transform2d(new Translation2d(), new Rotation2d(odoAngleOffset + (Math.PI * 0.5)))));
  }
  
  private String getFomattedPose() {
    var pose = odometer.getPoseMeters();
    return String.format(
            "(%.3f, %.3f) %.2f degrees",
            pose.getX(), pose.getY(), pose.getRotation().plus(new Rotation2d(-Math.PI/2.0)).getDegrees());
  }
  
  public void addDashboardWidgets(ShuffleboardTab tab) {
    tab.add("Field", field2d).withPosition(0, 0).withSize(6, 4);
    tab.addString("Pose", this::getFomattedPose).withPosition(6, 2).withSize(2, 1);
  }
}
