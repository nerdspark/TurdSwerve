// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax.IdleMode;

/** Add your docs here. */
public final class Constants {
    public static final int leftAzimuthID = 0;
    public static final int rightAzimuthID = 2;

    public static final int leftDriveID = 1;
    public static final int rightDriveID = 3;

    public static final int leftAbsoluteEncoderID = 0;
    public static final int rightAbsoluteEncoderID = 1;

    public static final int leftAzimuthEncoderAID = 0; // ?????

    public static final double leftAbsoluteEncoderOffset = 0;
    public static final double rightAbsoluteEncoderOffset = 0;

    public static final int azimuthRadiansPerPulse = 1;
    public static final int driveMetersPerPulse = 1;
    
    public static final int azimuthAmpLimit = 5;
    public static final int driveAmpLimit = 5;

    public static final IdleMode azimuthMode = IdleMode.kBrake;
    public static final IdleMode driveMode = IdleMode.kBrake;
}
