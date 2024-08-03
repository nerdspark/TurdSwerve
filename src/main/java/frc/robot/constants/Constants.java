// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;


/** Add your docs here. */
public final class Constants {
    public static final int driverPort = 1;
    
    /** CAN ID, Invert, Pod Positions, Offsets, Conversion Rates */
    public final class RobotMap {
        public static final double driveMetersPerMotorRotation = Units.inchesToMeters(2) * Math.PI / 1 / 0.36; //Wheel Diameter M * PI / Enc Count Per Rev / Gear Ratio
        public static final double driveMetersPerMotorRotationSpeed = Units.inchesToMeters(2) * Math.PI; //Used to convert motor rotations per second to meters per second

        public static final int pigeonID = 25;

        public static final int leftAzimuthID = 18;
        public static final int rightAzimuthID = 14;

        public static final int leftDriveID = 17;
        public static final int rightDriveID = 13;

        public static final int CAN_LeftAbsoluteEncoderID = 24;
        public static final int CAN_RightAbsoluteEncoderID = 22;

        
        public static final boolean leftAzimuthInvert = false;
        public static final boolean rightAzimuthInvert = false;
        public static final boolean leftDriveInvert = false;
        public static final boolean rightDriveInvert = false;
    }

    public final class RobotConfig {
        public static final PIDController gyroPID = new PIDController(0.046, 0d, 0.001);

        public static final double driveBaseRadius = Units.inchesToMeters(8.0);
        private static final double wheelBase = 12.5 / 2;
        private static final Translation2d leftPodPosition = new Translation2d(-Units.inchesToMeters(wheelBase), Units.inchesToMeters(wheelBase));
        private static final Translation2d rightPodPosition = new Translation2d(Units.inchesToMeters(wheelBase), -Units.inchesToMeters(wheelBase));
        public static final SwerveDriveKinematics drivetrainKinematics = new SwerveDriveKinematics(leftPodPosition, rightPodPosition);

        public static final double robotMaxSpeed = 4.5; //meters per second

        public static final double kDriveMotorGearRatio = 0.36;
        public static final double kWheelDiameterMeters = Units.inchesToMeters(2); // TODO: figure out right number
        public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
        public static final double kDriveEncoderTicks2Rot = 1d / 2048d;
        public static final double kDriveTicks2Meters = kDriveEncoderRot2Meter * kDriveEncoderTicks2Rot;


        // Azimuth Settings
        public static final boolean azimuthBrake = true;

        public static final int azimuthAmpLimit = 80;
        public static final double azimuthMaxOutput = 1;

        public static final double azimuthkP = 1.2;
        
        public static final double azimuthkI = 0.02;
        public static final double azimuthkD = 0.001;
        public static final double azimuthkS = 0.0;

        public static final double azimuthDriveSpeedMultiplier = 0.5;

        public static final double azimuthMotorRampRate = 0.0;

        // Drive Settings
        public static final double podMaxSpeed = 1;

        public static final boolean driveBrake = false;

        public static final int driveAmpLimit = 80;
        public static final int boostDriveLimit = 90;
        public static final double driveMotorRampRate = 0.2;

        public static final double azimuthRadiansPerMotorRotation = 2.200000047683716;

        public static final double devastatorLeftOffset = 0.316; //0.316
        public static final double devastatorRightOffset = -0.245 + 0.500; //-0.245
    }
}