// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.TurdPod;;

public class Drive extends SubsystemBase {
  /** Creates a new Drive. */
  public Drive() {
    TurdPod leftPod = TurdPod.TurdPod(
      Constants.leftAzimuthID,
      Constants.leftDriveID,
      Constants.leftAbsoluteEncoderID,
      Constants.leftAzimuthInvert,
      Constants.leftDriveInvert, 
      Constants.leftAbsoluteEncoderOffset);
    TurdPod rightPod = TurdPod(
      Constants.rightAzimuthID,
      Constants.rightDriveID,
      Constants.rightAbsoluteEncoderID,
      Constants.rightAzimuthInvert,
      Constants.rightDriveInvert, 
      Constants.rightAbsoluteEncoderOffset
    ); //TODO fix
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
