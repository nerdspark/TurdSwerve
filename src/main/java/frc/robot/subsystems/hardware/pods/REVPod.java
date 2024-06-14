// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.hardware.pods;

import frc.robot.subsystems.hardware.encoders.ThrifTurd;
import frc.robot.subsystems.hardware.motors.TurdMAX;

/** Sample REV pod with thrifty encoders and NEOs. Should represent normal NERDspark turdsweves */
public class REVPod extends TurdPod {
    private REVPod(int absoluteEncoderID, int azimuthID, int driveID, double absoluteEncoderOffset, boolean azimuthInvert, int azimuthLimit, double azimuthRadiansPerRot, boolean azimuthBrake, double azimuthRR, double kP, double kI, double kD, double Iz, double maxOut, boolean driveInvert, int driveLimit, boolean driveBrake, double driveRR) {
        absoluteEncoder = new ThrifTurd(absoluteEncoderID, absoluteEncoderOffset, azimuthRadiansPerRot);
        azimuthMotor = new TurdMAX(azimuthID, azimuthRadiansPerRot, azimuthLimit, azimuthInvert, azimuthBrake, 0.35, kP, kI, kD, Iz, maxOut);
        //drive conversion factor hardcoded to 1
        driveMotor = new TurdMAX(driveID, 1, driveLimit, driveInvert, driveBrake, driveRR);

        resetPod();
    }

    public REVPod(TurdConfig config) {
        this(config.absoluteEncoderID, config.azimuthID, config.driveID, config.absoluteEncoderOffset, config.azimuthInvert, config.azimuthLimit, config.azimuthRotationsPerTurn, config.azimuthBrake, config.azimuthRR, config.kP, config.kI, config.kD, config.wildcard, config.maxOut, config.driveInvert, config.driveLimit, config.driveBrake, config.driveRR);
        this.config = config;
        this.azimuthDriveSpeedMultiplier = config.driveSpeedMult;
    }

    @Override
    public void setPID(double IZone, double P, double I, double D, double outputRange, double ADMult) {
        azimuthMotor.setPID(IZone, P, I, D, outputRange);
        azimuthDriveSpeedMultiplier = ADMult;
    }
}
