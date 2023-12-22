// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class TurdSwerve extends SubsystemBase {
  private final Pigeon2 gyro = new Pigeon2(Constants.pigeonID);
  private final TurdPod leftPod = new TurdPod(Constants.leftAzimuthID, Constants.leftDriveID, Constants.leftAbsoluteEncoderID, Constants.leftAzimuthInvert, Constants.leftDriveInvert, Constants.leftAbsoluteEncoderOffset);
  private final TurdPod rightPod = new TurdPod(Constants.rightAzimuthID, Constants.rightDriveID, Constants.rightAbsoluteEncoderID, Constants.rightAzimuthInvert, Constants.rightDriveInvert, Constants.rightAbsoluteEncoderOffset);
  public TurdSwerve() {

  }

  public void setLeftPod(SwerveModuleState state) {
    leftPod.setPodState(state);
  }

  @Override
  public void periodic() {
    // SmartDashboard.putNumber("pigeon", gyro.getYaw());
  }
}
