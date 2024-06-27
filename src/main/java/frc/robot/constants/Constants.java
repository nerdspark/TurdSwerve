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

    public static final double robotMaxSpeed = 0.25; //meters per second
    
    /** CAN ID, Invert, Pod Positions, Offsets, Conversion Rates */
    public final class RobotMap {
        public static final double driveMetersPerMotorRotation = Units.inchesToMeters(2) * Math.PI * 33 / 45 / 15/13;

        
        public static final int pigeonID = 25;


        public static final int leftAzimuthID = 18;
        public static final int rightAzimuthID = 14;

        public static final int leftDriveID = 17;
        public static final int rightDriveID = 13;


        public static final int RIO_LeftAbsoluteEncoderID = 3;
        public static final int RIO_RightAbsoluteEncoderID = 0;

        private static final int CAN_LeftAbsoluteEncoderID = 24;
        private static final int CAN_RightAbsoluteEncoderID = 22;

        
        public static final boolean leftAzimuthInvert = false;
        public static final boolean rightAzimuthInvert = false;
        public static final boolean leftDriveInvert = false;
        public static final boolean rightDriveInvert = true;
    }

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

        private static final double azimuthRadiansPerMotorRotation = 15/33;

        private static final double brownoutLeftOffset = 4.731349179724511;//absolute encoder reading at position
        private static final double brownoutRightOffset = 0.73436864196419 + Math.PI;// gears facing inwards: fwd/bck TODO: less janky alignment


        private static final TurdConfig NeoTemplate = new TurdConfig(azimuthAmpLimit, azimuthRadiansPerMotorRotation, azimuthBrake, azimuthMotorRampRate, azimuthkP, azimuthkI, azimuthkD, azimuthkIz, azimuthMaxOutput, driveAmpLimit, boostDriveLimit, driveBrake, driveMotorRampRate, azimuthDriveSpeedMultiplier, PodType.REV); 

        private static final TurdConfig BrownoutLeftPod = new TurdConfig(RobotMap.RIO_LeftAbsoluteEncoderID, RobotMap.leftAzimuthID, RobotMap.leftAzimuthInvert, RobotMap.leftDriveID, RobotMap.leftDriveInvert, brownoutLeftOffset, NeoTemplate);
        private static final TurdConfig BrownoutRightPod = new TurdConfig(RobotMap.RIO_RightAbsoluteEncoderID, RobotMap.rightAzimuthID, RobotMap.rightAzimuthInvert,RobotMap.rightDriveID, RobotMap.rightDriveInvert, brownoutRightOffset, NeoTemplate);
        
        public static final MultiTurd Brownout = new MultiTurd(gyroPID, RobotMap.pigeonID, drivetrainKinematics, podMaxSpeed, NeoTemplate, new TurdConfig[] {BrownoutLeftPod, BrownoutRightPod});
    }

     //black and white
     public final class ProwlConfig {
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
            
        private static final double brownoutLeftOffset = 1.01;
        private static final double brownoutRightOffset = 1.06;


        private static final TurdConfig NeoTemplate = new TurdConfig(azimuthAmpLimit, azimuthRadiansPerMotorRotation, azimuthBrake, azimuthMotorRampRate, azimuthkP, azimuthkI, azimuthkD, azimuthkIz, azimuthMaxOutput, driveAmpLimit, boostDriveLimit, driveBrake, driveMotorRampRate, azimuthDriveSpeedMultiplier, PodType.REV); 

        private static final TurdConfig ProwlLeftPod = new TurdConfig(RobotMap.RIO_LeftAbsoluteEncoderID, RobotMap.leftAzimuthID, RobotMap.leftAzimuthInvert, RobotMap.leftDriveID, RobotMap.leftDriveInvert, brownoutLeftOffset, NeoTemplate);
        private static final TurdConfig ProwlRightPod = new TurdConfig(RobotMap.RIO_RightAbsoluteEncoderID, RobotMap.rightAzimuthID, RobotMap.rightAzimuthInvert,RobotMap.rightDriveID, RobotMap.rightDriveInvert, brownoutRightOffset, NeoTemplate);
        
        public static final MultiTurd Brownout = new MultiTurd(gyroPID, RobotMap.pigeonID, drivetrainKinematics, podMaxSpeed, NeoTemplate, new TurdConfig[] {ProwlLeftPod, ProwlRightPod});
    }

    //purple
    public final class SkywarpConfig {
        private static final PIDController gyroPID = new PIDController(0, 0d, 0d);

        private static final double wheelBase = 5.75;
        private static final Translation2d leftPodPosition = new Translation2d(-Units.inchesToMeters(wheelBase), Units.inchesToMeters(wheelBase));
        private static final Translation2d rightPodPosition = new Translation2d(Units.inchesToMeters(wheelBase), -Units.inchesToMeters(wheelBase));
        private static final SwerveDriveKinematics drivetrainKinematics = new SwerveDriveKinematics(leftPodPosition, rightPodPosition);


        // Azimuth Settings
        private static final boolean azimuthBrake = true;

        private static final int azimuthAmpLimit = 25;
        private static final double azimuthMaxOutput = 1;

        private static final double azimuthkP = 2;
        
        private static final double azimuthkI = 0.0;
        private static final double azimuthkD = 0.0;
        private static final double azimuthkS = 0;

        private static final double azimuthDriveSpeedMultiplier = 0;//0.5;

        private static final double azimuthMotorRampRate = 0.0;

        // Drive Settings
        private static final double podMaxSpeed = 1;

        private static final boolean driveBrake = false;

        private static final int driveAmpLimit = 35;
        private static final int boostDriveLimit = 90;
        private static final double driveMotorRampRate = 0.2;

        private static final double azimuthRadiansPerMotorRotation = 33d/15d;

        private static final double skywarpLeftOffset = 0.163; 
        private static final double skywarpRightOffset = -0.422;


        private static final TurdConfig SkywarpTemplate = new TurdConfig(azimuthAmpLimit, azimuthRadiansPerMotorRotation, azimuthBrake, azimuthMotorRampRate, azimuthkP, azimuthkI, azimuthkD, azimuthkS, azimuthMaxOutput, driveAmpLimit, boostDriveLimit, driveBrake, driveMotorRampRate, azimuthDriveSpeedMultiplier, PodType.CTRE); 

        private static final TurdConfig leftPod = new TurdConfig(RobotMap.CAN_LeftAbsoluteEncoderID, RobotMap.leftAzimuthID, RobotMap.leftAzimuthInvert, RobotMap.leftDriveID, RobotMap.leftDriveInvert, skywarpLeftOffset, SkywarpTemplate);
        private static final TurdConfig rightPod = new TurdConfig(RobotMap.CAN_RightAbsoluteEncoderID, RobotMap.rightAzimuthID, RobotMap.rightAzimuthInvert,RobotMap.rightDriveID, RobotMap.rightDriveInvert, skywarpRightOffset, SkywarpTemplate);
        
        public static final MultiTurd Skywarp = new MultiTurd(gyroPID, RobotMap.pigeonID, drivetrainKinematics, podMaxSpeed, SkywarpTemplate, new TurdConfig[] {leftPod, rightPod});
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
        private static final double azimuthkP = 1.0;
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
        private static final int pigeonID = 25;

        private static final int frontLeftAzimuthID  = 2;
        private static final int frontRightAzimuthID = 3;
        private static final int backLeftAzimuthID   = 4;
        private static final int backRightAzimuthID  = 5;


        private static final int frontLeftDriveID  = 6;
        private static final int frontRightDriveID = 7;
        private static final int backLeftDriveID   = 8;
        private static final int backRightDriveID  = 9;

        private static final int frontLeftAbsoluteEncoderID  = 2;
        private static final int frontRightAbsoluteEncoderID = 3;
        private static final int backLeftAbsoluteEncoderID   = 1;
        private static final int backRightAbsoluteEncoderID  = 0;

        
        private static final boolean frontLeftAzimuthInvert  = false;
        private static final boolean frontRightAzimuthInvert = false;
        private static final boolean backLeftAzimuthInvert   = false;
        private static final boolean backRightAzimuthInvert  = false;

        private static final boolean frontLeftDriveInvert  = false;
        private static final boolean frontRightDriveInvert = true;
        private static final boolean backLeftDriveInvert   = false;
        private static final boolean backRightDriveInvert  = true;

        
        private static final double frontLeftAbsoluteEncoderOffset  = 0.94d; //absolute encoder reading at position
        private static final double frontRightAbsoluteEncoderOffset = 0.37d; //absolute encoder reading at position
        private static final double backLeftAbsoluteEncoderOffset   = 0.45d; //absolute encoder reading at position
        private static final double backRightAbsoluteEncoderOffset  = 0.76d; // gears facing inwards: fwd/bck



        // Megatron config
        private static final TurdConfig MegatronTemplate = new TurdConfig(azimuthAmpLimit, azimuthRadiansPerMotorRotation, azimuthBrake, azimuthMotorRampRate, azimuthkP, azimuthkI, azimuthkD, azimuthkFF, azimuthMaxOutput, driveAmpLimit, boostDriveLimit, driveBrake, driveMotorRampRate, azimuthDriveSpeedMultiplier, PodType.Megatron); 

        private static final TurdConfig frontLeftPod = new TurdConfig(frontLeftAbsoluteEncoderID, frontLeftAzimuthID, frontLeftAzimuthInvert, frontLeftDriveID, frontLeftDriveInvert, frontLeftAbsoluteEncoderOffset, MegatronTemplate);
        private static final TurdConfig frontRightPod = new TurdConfig(frontRightAbsoluteEncoderID, frontRightAzimuthID, frontRightAzimuthInvert, frontRightDriveID, frontRightDriveInvert, frontRightAbsoluteEncoderOffset, MegatronTemplate);
        private static final TurdConfig backLeftPod = new TurdConfig(backLeftAbsoluteEncoderID, backLeftAzimuthID, backLeftAzimuthInvert, backLeftDriveID, backLeftDriveInvert, backLeftAbsoluteEncoderOffset, MegatronTemplate);
        private static final TurdConfig backRightPod = new TurdConfig(backRightAbsoluteEncoderID, backRightAzimuthID, backRightAzimuthInvert, backRightDriveID, backRightDriveInvert, backRightAbsoluteEncoderOffset, MegatronTemplate);
        
        public static final MultiTurd Megatron = new MultiTurd(gyroPID, pigeonID, drivetrainKinematics, podMaxSpeed, MegatronTemplate, new TurdConfig[] {frontLeftPod, frontRightPod, backLeftPod, backRightPod});
    
    }

}
