// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.hardware.pods;

import frc.robot.subsystems.hardware.encoders.CANTurd;
import frc.robot.subsystems.hardware.motors.TurdonFX;

/** This is a sample pod that uses a CANcoder and TalonFXes. */
public class CTREPod extends TurdPod {
    public CTREPod(int azimuthID, int driveID, int absoluteEncoderID, boolean azimuthInvert, boolean driveInvert, double absoluteEncoderOffset) {
        absoluteEncoder = new CANTurd(absoluteEncoderID, absoluteEncoderOffset);
        azimuthMotor = new TurdonFX(azimuthID, azimuthInvert, false, 0d, 0d, 0d, 0d);
        driveMotor = new TurdonFX(driveID, driveInvert, false, 0d, 0d, 0d, 0d);
        resetPod();
    }

    @Override
    public void setPID(double kS, double P, double I, double D, double outputRange, double ADMult) {
        azimuthMotor.setPID(kS, P, I, D, outputRange);
        azimuthDriveSpeedMultiplier = ADMult;
    }
}
