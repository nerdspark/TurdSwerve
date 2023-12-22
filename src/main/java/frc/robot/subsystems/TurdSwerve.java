// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;
import com.ctre.phoenix.sensors.Pigeon2Configuration;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
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
  public TurdSwerve() {
    // gyro.configAllSettings(new Pigeon2Configuration());
  }

  public void resetOdometry(Pose2d pose) {
    odometer.resetPosition(new Rotation2d(gyro.getYaw()), new SwerveModulePosition[] {leftPod.getPodPosition(), rightPod.getPodPosition()}, pose);
  }

  public void setLeftPod(SwerveModuleState state) {
    leftPod.setPodState(state);
  }

  public void setRobotSpeeds(ChassisSpeeds chassisSpeeds) {
    SwerveModuleState[] states = Constants.drivetrainKinematics.toSwerveModuleStates(chassisSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.podMaxSpeed);

    leftPod.setPodState(states[0]);
    rightPod.setPodState(states[1]);
  }

  @Override
  public void periodic() {
    odometer.update(new Rotation2d(gyro.getYaw()), new SwerveModulePosition[] {leftPod.getPodPosition(), rightPod.getPodPosition()});
    SmartDashboard.putNumber("pigeon", gyro.getYaw());
  }
}
