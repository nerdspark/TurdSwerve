// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import java.util.HashMap;
import java.util.Hashtable;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public final class Constants {
    public static final int driverPort = 0;

    public static final double gyroP = 2;
    public static final double gyroI = 0.0;
    public static final double gyroD = 0.00;

    public static final double robotMaxSpeed = 0.25;
    public static final double podMaxSpeed = 1;


    // Azimuth Settings
    public static final IdleMode azimuthMode = IdleMode.kBrake;

    public static final int azimuthAmpLimit = 35;
    public static final double azimuthMaxOutput = 0.25;

    public static final double azimuthkP = 0.4;
    public static final double azimuthkI = 0.0;
    public static final double azimuthkD = 0.003;
    public static final double azimuthkIz = 0;
    public static final double azimuthDriveSpeedMultiplier = 0;//0.5;

    // Drive Settings
    public static final IdleMode driveMode = IdleMode.kCoast;

    public static final int driveAmpLimit = 25;
    public static final int driveTopAmpLimit = 90;
    public static final double driveSpeedToPower = 1.0;
    public static final double driveMotorRampRate = 0.2;
}