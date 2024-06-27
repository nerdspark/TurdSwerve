// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.hardware.pods;

import frc.robot.subsystems.hardware.encoders.CANTurd;
import frc.robot.subsystems.hardware.encoders.ThrifTurd;
import frc.robot.subsystems.hardware.motors.TurdonFX;

/** This is a sample pod that uses a CANcoder and TalonFXes. */
public class MegatronPod extends TurdPod {
    private MegatronPod(int absoluteEncoderID, int azimuthID, int driveID, double absoluteEncoderOffset, boolean azimuthInvert, int azimuthLimit, double azimuthRotationsPerRot, boolean azimuthBrake, double azimuthRR, double kP, double kI, double kD, double FF, double maxOut, double ADMult, boolean driveInvert, int driveLimit, boolean driveBrake, double driveRR) {
        absoluteEncoder = new ThrifTurd(absoluteEncoderID, absoluteEncoderOffset, 1);
        azimuthMotor = new TurdonFX(azimuthID, azimuthInvert, azimuthBrake, azimuthLimit, azimuthRR, 1d, azimuthRotationsPerRot, kP, kI, kD, FF, maxOut, absoluteEncoderID);
        driveMotor = new TurdonFX(driveID, driveInvert, driveBrake, driveLimit, driveRR, 1d, 1d);
        resetPod();
    }

    public MegatronPod(TurdConfig config) {
        this(config.absoluteEncoderID, config.azimuthID, config.driveID, config.absoluteEncoderOffset, config.azimuthInvert, config.azimuthLimit, config.azimuthRotationsPerTurn, config.azimuthBrake, config.azimuthRR, config.kP, config.kI, config.kD, config.wildcard, config.maxOut, config.driveSpeedMult, config.driveInvert, config.driveLimit, config.driveBrake, config.driveRR);
        this.config = config;
        this.azimuthDriveSpeedMultiplier = config.driveSpeedMult;
    }
    
    @Override
    public void setPID(double kS, double P, double I, double D, double outputRange, double ADMult) {
        azimuthMotor.setPID(kS, P, I, D, outputRange);
        azimuthDriveSpeedMultiplier = ADMult;
    }
}
