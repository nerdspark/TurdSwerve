// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.MultiTurd;
import frc.robot.subsystems.hardware.pods.TurdConfig;
import frc.robot.subsystems.hardware.pods.TurdConfig.PodType;


/** Add your docs here. */
public final class Constants {
    public static final int driverPort = 0;

    public static final double robotMaxSpeed = 0.25;

    //OG Turdswerve
    public final class BrownoutConfig {
        private static final PIDController gyroPID = new PIDController(2, 0d, 0d);

        private static final Translation2d robotCenter = new Translation2d(0, 0); // serves as "center of robot for calculations; robot will turn about this point
        private static final double wheelBase = 6.25;
        private static final Translation2d leftPodPosition = new Translation2d(-Units.inchesToMeters(wheelBase), -Units.inchesToMeters(wheelBase)); // units in meters
        private static final Translation2d rightPodPosition = new Translation2d(Units.inchesToMeters(wheelBase), Units.inchesToMeters(wheelBase));
        private static final SwerveDriveKinematics drivetrainKinematics = new SwerveDriveKinematics(robotCenter.minus(leftPodPosition), robotCenter.minus(rightPodPosition));


        // Azimuth Settings
        private static final boolean azimuthBrake = true;

        private static final int azimuthAmpLimit = 35;
        private static final double azimuthMaxOutput = 0.25;

        private static final double azimuthkP = 0.4;
        private static final double azimuthkI = 0.0;
        private static final double azimuthkD = 0.003;
        private static final double azimuthkIz = 0;

        private static final double azimuthDriveSpeedMultiplier = 0;//0.5;

        private static final double azimuthMotorRampRate = 0.35;


        // Drive Settings
        private static final double podMaxSpeed = 1;

        private static final boolean driveBrake = false;

        private static final int driveAmpLimit = 25;
        private static final int boostDriveLimit = 90;
        private static final double driveMotorRampRate = 0.2;

        private static final double azimuthRadiansPerMotorRotation = 2*Math.PI*15/33;


        // robot map
        private static final int pigeonID = 6;

        private static final int leftAzimuthID = 2;
        private static final int rightAzimuthID = 3;

        private static final int leftDriveID = 4;
        private static final int rightDriveID = 5;

        private static final int leftAbsoluteEncoderID = 3;
        private static final int rightAbsoluteEncoderID = 0;
        
        private static final boolean leftAzimuthInvert = false;
        private static final boolean rightAzimuthInvert = false;
        private static final boolean leftDriveInvert = false;
        private static final boolean rightDriveInvert = true;

        
        private static final double brownoutLeftOffset = 4.731349179724511;//absolute encoder reading at position
        private static final double brownoutRightOffset = 0.73436864196419 + Math.PI;// gears facing inwards: fwd/bck TODO: less janky alignment


        private static final TurdConfig NeoTemplate = new TurdConfig(azimuthAmpLimit, azimuthRadiansPerMotorRotation, azimuthBrake, azimuthMotorRampRate, azimuthkP, azimuthkI, azimuthkD, azimuthkIz, azimuthMaxOutput, driveAmpLimit, boostDriveLimit, driveBrake, driveMotorRampRate, azimuthDriveSpeedMultiplier, PodType.REV); 

        private static final TurdConfig BrownoutLeftPod = new TurdConfig(leftAbsoluteEncoderID, leftAzimuthID, leftAzimuthInvert, leftDriveID, leftDriveInvert, brownoutLeftOffset, NeoTemplate);
        private static final TurdConfig BrownoutRightPod = new TurdConfig(rightAbsoluteEncoderID, rightAzimuthID, rightAzimuthInvert, rightDriveID, rightDriveInvert, brownoutRightOffset, NeoTemplate);
        
        public static final MultiTurd Brownout = new MultiTurd(gyroPID, pigeonID, drivetrainKinematics, podMaxSpeed, NeoTemplate, new TurdConfig[] {BrownoutLeftPod, BrownoutRightPod});
    }

    //purple
    public final class SkywarpConfig {
        private static final PIDController gyroPID = new PIDController(0, 0d, 0d);

        private static final Translation2d robotCenter = new Translation2d(0, 0); // serves as "center of robot for calculations; robot will turn about this point
        private static final double wheelBase = 5.75;
        private static final Translation2d leftPodPosition = new Translation2d(-Units.inchesToMeters(wheelBase), -Units.inchesToMeters(wheelBase)); // units in meters
        private static final Translation2d rightPodPosition = new Translation2d(Units.inchesToMeters(wheelBase), Units.inchesToMeters(wheelBase));
        private static final SwerveDriveKinematics drivetrainKinematics = new SwerveDriveKinematics(robotCenter.minus(leftPodPosition), robotCenter.minus(rightPodPosition));


        // Azimuth Settings
        private static final boolean azimuthBrake = true;

        private static final int azimuthAmpLimit = 35;
        private static final double azimuthMaxOutput = 1;

        private static final double azimuthkP = 0.4;
        private static final double azimuthkI = 0.0;
        private static final double azimuthkD = 0.003;
        private static final double azimuthkIz = 0;

        private static final double azimuthDriveSpeedMultiplier = 0;//0.5;

        private static final double azimuthMotorRampRate = 0.35;


        // Drive Settings
        private static final double podMaxSpeed = 1;

        private static final boolean driveBrake = false;

        private static final int driveAmpLimit = 25;
        private static final int boostDriveLimit = 90;
        private static final double driveMotorRampRate = 0.2;

        private static final double azimuthRadiansPerMotorRotation = 15/33;


        // robot map
        private static final int pigeonID = 6;

        private static final int leftAzimuthID = 2;
        private static final int rightAzimuthID = 3;

        private static final int leftDriveID = 4;
        private static final int rightDriveID = 5;

        private static final int leftAbsoluteEncoderID = 7;
        private static final int rightAbsoluteEncoderID = 8;
        
        private static final boolean leftAzimuthInvert = false;
        private static final boolean rightAzimuthInvert = false;
        private static final boolean leftDriveInvert = false;
        private static final boolean rightDriveInvert = true;

        
        private static final double skywarpLeftOffset = 0.4863;//absolute encoder reading at position
        private static final double skywarpRightOffset = -0.431;// gears facing inwards: fwd/bck TODO: less janky alignment


        private static final TurdConfig SkywarpTemplate = new TurdConfig(azimuthAmpLimit, azimuthRadiansPerMotorRotation, azimuthBrake, azimuthMotorRampRate, azimuthkP, azimuthkI, azimuthkD, azimuthkIz, azimuthMaxOutput, driveAmpLimit, boostDriveLimit, driveBrake, driveMotorRampRate, azimuthDriveSpeedMultiplier, PodType.CTRE); 

        private static final TurdConfig leftPod = new TurdConfig(leftAbsoluteEncoderID, leftAzimuthID, leftAzimuthInvert, leftDriveID, leftDriveInvert, skywarpLeftOffset, SkywarpTemplate);
        private static final TurdConfig rightPod = new TurdConfig(rightAbsoluteEncoderID, rightAzimuthID, rightAzimuthInvert, rightDriveID, rightDriveInvert, skywarpRightOffset, SkywarpTemplate);
        
        public static final MultiTurd Skywarp = new MultiTurd(gyroPID, pigeonID, drivetrainKinematics, podMaxSpeed, SkywarpTemplate, new TurdConfig[] {leftPod, rightPod});
    }

    public final class MegatronConfig {
        //TODO: tune
        private static final PIDController gyroPID = new PIDController(0, 0d, 0d);

        private static final Translation2d robotCenter = new Translation2d(0, 0); // serves as "center of robot for calculations; robot will turn about this point
        private static final double wheelBase = 6.375;
        private static final Translation2d frontLeftPodPosition = new Translation2d(-Units.inchesToMeters(wheelBase), -Units.inchesToMeters(wheelBase)); // units in meters
        private static final Translation2d frontRightPodPosition = new Translation2d(Units.inchesToMeters(wheelBase), -Units.inchesToMeters(wheelBase)); // units in meters
        private static final Translation2d backLeftPodPosition = new Translation2d(-Units.inchesToMeters(wheelBase), Units.inchesToMeters(wheelBase)); // units in meters
        private static final Translation2d backRightPodPosition = new Translation2d(Units.inchesToMeters(wheelBase), Units.inchesToMeters(wheelBase));
        private static final SwerveDriveKinematics drivetrainKinematics = new SwerveDriveKinematics(robotCenter.minus(frontLeftPodPosition), robotCenter.minus(frontRightPodPosition), robotCenter.minus(backLeftPodPosition), robotCenter.minus(backRightPodPosition));


        // Azimuth Settings
        private static final boolean azimuthBrake = true;

        private static final int azimuthAmpLimit = 35;
        private static final double azimuthMaxOutput = 0.25;

        //TODO: tune
        private static final double azimuthkP = 0.0;
        private static final double azimuthkI = 0.0;
        private static final double azimuthkD = 0.0;
        private static final double azimuthkFF = 0;

        private static final double azimuthDriveSpeedMultiplier = 0;//0.5;

        private static final double azimuthMotorRampRate = 0.35;


        // Drive Settings
        private static final double podMaxSpeed = 1;

        private static final boolean driveBrake = false;

        private static final int driveAmpLimit = 25;
        private static final int boostDriveLimit = 90;
        private static final double driveMotorRampRate = 0.2;

        private static final double azimuthRadiansPerMotorRotation = 2*Math.PI*15/33;


        // robot map
        private static final int pigeonID = 14;

        private static final int frontLeftAzimuthID  = 2;
        private static final int frontRightAzimuthID = 3;
        private static final int backLeftAzimuthID   = 4;
        private static final int backRightAzimuthID  = 5;


        private static final int frontLeftDriveID  = 6;
        private static final int frontRightDriveID = 7;
        private static final int backLeftDriveID   = 8;
        private static final int backRightDriveID  = 9;

        private static final int frontLeftAbsoluteEncoderID  = 10;
        private static final int frontRightAbsoluteEncoderID = 11;
        private static final int backRightAbsoluteEncoderID  = 12;
        private static final int backLeftAbsoluteEncoderID   = 13;

        
        private static final boolean frontLeftAzimuthInvert  = false;
        private static final boolean frontRightAzimuthInvert = false;
        private static final boolean backLeftAzimuthInvert   = false;
        private static final boolean backRightAzimuthInvert  = false;

        private static final boolean frontLeftDriveInvert  = false;
        private static final boolean frontRightDriveInvert = true;
        private static final boolean backLeftDriveInvert   = false;
        private static final boolean backRightDriveInvert  = true;

        
        private static final double frontLeftAbsoluteEncoderOffset  = 0d; //absolute encoder reading at position
        private static final double frontRightAbsoluteEncoderOffset = 0d; //absolute encoder reading at position
        private static final double backLeftAbsoluteEncoderOffset   = 0d; //absolute encoder reading at position
        private static final double backRightAbsoluteEncoderOffset  = 0d; // gears facing inwards: fwd/bck



        // Megatron config
        private static final TurdConfig MegatronTemplate = new TurdConfig(azimuthAmpLimit, azimuthRadiansPerMotorRotation, azimuthBrake, azimuthMotorRampRate, azimuthkP, azimuthkI, azimuthkD, azimuthkFF, azimuthMaxOutput, driveAmpLimit, boostDriveLimit, driveBrake, driveMotorRampRate, azimuthDriveSpeedMultiplier, PodType.REV); 

        private static final TurdConfig frontLeftPod = new TurdConfig(frontLeftAbsoluteEncoderID, frontLeftAzimuthID, frontLeftAzimuthInvert, frontLeftDriveID, frontLeftDriveInvert, frontLeftAbsoluteEncoderOffset, MegatronTemplate);
        private static final TurdConfig frontRightPod = new TurdConfig(frontRightAbsoluteEncoderID, frontRightAzimuthID, frontRightAzimuthInvert, frontRightDriveID, frontRightDriveInvert, frontRightAbsoluteEncoderOffset, MegatronTemplate);
        private static final TurdConfig backLeftPod = new TurdConfig(backLeftAbsoluteEncoderID, backLeftAzimuthID, backLeftAzimuthInvert, backLeftDriveID, backLeftDriveInvert, backLeftAbsoluteEncoderOffset, MegatronTemplate);
        private static final TurdConfig backRightPod = new TurdConfig(backRightAbsoluteEncoderID, backRightAzimuthID, backRightAzimuthInvert, backRightDriveID, backRightDriveInvert, backRightAbsoluteEncoderOffset, MegatronTemplate);
        
        public static final MultiTurd Megatron = new MultiTurd(gyroPID, pigeonID, drivetrainKinematics, podMaxSpeed, MegatronTemplate, new TurdConfig[] {frontLeftPod, frontRightPod, backLeftPod, backRightPod});
    
    }

}
