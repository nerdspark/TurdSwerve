// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.hardware.pods;

public class TurdConfig {
    public int absoluteEncoderID;
    public int azimuthID;
    public int driveID;
    public double absoluteEncoderOffset;
    public boolean azimuthInvert;
    public int azimuthLimit;
    public double azimuthRadiansPerRot;
    public boolean azimuthBrake;
    public double azimuthRR;
    public double kP;
    public double kI;
    public double kD;
    public double wildcard;
    public double maxOut;
    public boolean driveInvert;
    public int driveLimit;
    public int boostDriveLimit;
    public boolean driveBrake;
    public double driveRR;
    public double driveSpeedMult;
    public PodType podType = PodType.undef;

    /**
     * @implNote template constructor - do not supply this to a TurdPod
     */
    public TurdConfig(int azimuthLimit, double azimuthRadiansPerRot, boolean azimuthBrake, double azimuthRR, double kP, double kI, double kD, double wildcard, double maxOut, int driveLimit, int boostDriveLimit, boolean driveBrake, double driveRR, double driveSpeedMult, PodType podType) {
        this.azimuthLimit = azimuthLimit;
        this.azimuthRadiansPerRot = azimuthRadiansPerRot;
        this.azimuthBrake = azimuthBrake;
        this.azimuthRR = azimuthRR;
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.wildcard = wildcard;
        this.maxOut = maxOut;
        this.driveLimit = driveLimit;
        this.boostDriveLimit = boostDriveLimit;
        this.driveBrake = driveBrake;
        this.driveRR = driveRR;
        this.driveSpeedMult = driveSpeedMult;
        this.podType = podType;
    }

    /**
     * @ImplNote this constructor is used to create a new TurdConfig from a pre-existing template
     */
    public TurdConfig(int absoluteEncoderID, int azimuthID, boolean azimuthInvert, int driveID, boolean driveInvert, double absoluteEncoderOffset, TurdConfig config) {
        this(config.azimuthLimit, config.azimuthRadiansPerRot, config.azimuthBrake, config.azimuthRR, config.kP, config.kI, config.kD, config.wildcard, config.maxOut, config.driveLimit, config.boostDriveLimit, config.driveBrake, config.driveRR, config.driveSpeedMult, config.podType);        
        
        this.absoluteEncoderID = absoluteEncoderID;
        this.azimuthID = azimuthID;
        this.driveID = driveID;
        this.absoluteEncoderOffset = absoluteEncoderOffset;
        this.azimuthInvert = azimuthInvert;
        
    }


    /**
     * @ImplNote this constructor is used to create a new TurdConfig from a pre-existing template
     */
    public TurdConfig(int absoluteEncoderID, int azimuthID, boolean azimuthInvert, int driveID, boolean driveInvert, double absoluteEncoderOffset, double kP, double kI, double kD, double wildcard, TurdConfig config) {
        this(config.azimuthLimit, config.azimuthRadiansPerRot, config.azimuthBrake, config.azimuthRR, kP, kI, kD, wildcard, config.maxOut, config.driveLimit, config.boostDriveLimit, config.driveBrake, config.driveRR, config.driveSpeedMult, config.podType);        
        
        this.absoluteEncoderID = absoluteEncoderID;
        this.azimuthID = azimuthID;
        this.driveID = driveID;
        this.absoluteEncoderOffset = absoluteEncoderOffset;
        this.azimuthInvert = azimuthInvert;
        
    }

    public static enum PodType {
        REV, CTRE, undef
    }

}
