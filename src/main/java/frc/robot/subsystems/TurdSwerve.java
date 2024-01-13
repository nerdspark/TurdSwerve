// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;
import com.ctre.phoenix.sensors.Pigeon2Configuration;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
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
  private double targetAngle = 0;

  private Rotation2d gyroResetAngle = new Rotation2d();
  public TurdSwerve() {
    // gyro.configAllSettings(new Pigeon2Configuration());
  }

  public void resetOdometry(Pose2d pose) {
    odometer.resetPosition(getGyro(), new SwerveModulePosition[] {leftPod.getPodPosition(), rightPod.getPodPosition()}, pose);
  }

  public void resetPods() {
    resetGyro();
    leftPod.resetPod();
    rightPod.resetPod();
    leftPod.setPID(azimuthP.getDouble(Constants.azimuthkP), azimuthI.getDouble(Constants.azimuthkI), azimuthD.getDouble(Constants.azimuthkD), azimuthIzone.getDouble(Constants.azimuthkIz), Constants.azimuthMaxOutput, ADMult.getDouble(Constants.azimuthDriveSpeedMultiplier));
    rightPod.setPID(azimuthP.getDouble(Constants.azimuthkP), azimuthI.getDouble(Constants.azimuthkI), azimuthD.getDouble(Constants.azimuthkD), azimuthIzone.getDouble(Constants.azimuthkIz), Constants.azimuthMaxOutput, ADMult.getDouble(Constants.azimuthDriveSpeedMultiplier));
  }

  public Rotation2d getGyro() {
    return new Rotation2d(gyro.getYaw()*Math.PI/180).minus(gyroResetAngle);
  }

  public void resetGyro() {
    gyroResetAngle = getGyro().plus(gyroResetAngle);
    targetAngle = 0;
  }

  public void setLeftPod(SwerveModuleState state) {
    leftPod.setPodState(state);
  }

  public void setRobotSpeeds(ChassisSpeeds chassisSpeeds) {
    targetAngle += chassisSpeeds.omegaRadiansPerSecond / 50;
    chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond, GyroPID.calculate(getGyro().getRadians(), targetAngle), getGyro());
    SwerveModuleState[] states = Constants.drivetrainKinematics.toSwerveModuleStates(chassisSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.podMaxSpeed);

    leftPod.setPodState(states[0]);
    rightPod.setPodState(states[1]);
  }

  @Override
  public void periodic() {
    odometer.update(getGyro(), new SwerveModulePosition[] {leftPod.getPodPosition(), rightPod.getPodPosition()});
    SmartDashboard.putNumber("pigeon", getGyro().getDegrees());
  }
}
