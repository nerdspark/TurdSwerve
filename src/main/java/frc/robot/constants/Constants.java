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
    public static final int driverPort = 0;
    
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

        public static final int CAN_LeftAbsoluteEncoderID = 24;
        public static final int CAN_RightAbsoluteEncoderID = 22;

        
        public static final boolean leftAzimuthInvert = false;
        public static final boolean rightAzimuthInvert = false;
        public static final boolean leftDriveInvert = false;
        public static final boolean rightDriveInvert = true;
    }

    //purple
    public final class RobotConfig {
        public static final PIDController gyroPID = new PIDController(0, 0d, 0d);

        private static final double wheelBase = 5.75;
        private static final Translation2d leftPodPosition = new Translation2d(-Units.inchesToMeters(wheelBase), Units.inchesToMeters(wheelBase));
        private static final Translation2d rightPodPosition = new Translation2d(Units.inchesToMeters(wheelBase), -Units.inchesToMeters(wheelBase));
        public static final SwerveDriveKinematics drivetrainKinematics = new SwerveDriveKinematics(leftPodPosition, rightPodPosition);

        public static final double robotMaxSpeed = 0.25; //meters per second


        // Azimuth Settings
        public static final boolean azimuthBrake = true;

        public static final int azimuthAmpLimit = 25;
        public static final double azimuthMaxOutput = 1;

        public static final double azimuthkP = 1.2;
        
        public static final double azimuthkI = 0.02;
        public static final double azimuthkD = 0.001;
        public static final double azimuthkS = 0.0;

        public static final double azimuthDriveSpeedMultiplier = 0;//0.5;

        public static final double azimuthMotorRampRate = 0.0;

        // Drive Settings
        public static final double podMaxSpeed = 1;

        public static final boolean driveBrake = false;

        public static final int driveAmpLimit = 35;
        public static final int boostDriveLimit = 90;
        public static final double driveMotorRampRate = 0.2;

        public static final double azimuthRadiansPerMotorRotation = 33d/15d;

        public static final double skywarpLeftOffset = 0.163; 
        public static final double skywarpRightOffset = -0.422;
    }
}
