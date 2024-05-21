// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.hardware.pods;

import frc.robot.constants.Constants;
import frc.robot.constants.RobotMap;
import frc.robot.subsystems.hardware.encoders.ThrifTurd;
import frc.robot.subsystems.hardware.motors.TurdMAX;

/** Sample REV pod with thrifty encoders and NEOs. Should represent normal NERDspark turdsweves */
public class REVPod extends TurdPod {
    public REVPod(int azimuthID, int driveID, int absoluteEncoderID, boolean azimuthInvert, boolean driveInvert, double absoluteEncoderOffset) {
        absoluteEncoder = new ThrifTurd(absoluteEncoderID, absoluteEncoderOffset, RobotMap.absoluteRadiansPerEncoderRotation);
        azimuthMotor = new TurdMAX(RobotMap.azimuthRadiansPerMotorRotation, Constants.azimuthAmpLimit, azimuthInvert, Constants.azimuthMode, 0.35);
        resetPod();
    }

    @Override
    public void setPID(double IZone, double P, double I, double D, double outputRange, double ADMult) {
        azimuthMotor.setPID(IZone, P, I, D, outputRange);
        azimuthDriveSpeedMultiplier = ADMult;
    }
}
