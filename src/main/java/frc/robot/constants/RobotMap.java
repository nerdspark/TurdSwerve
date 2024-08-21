// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/** CAN ID, Invert, Pod Positions, Offsets, Conversion Rates */
public final class RobotMap {
    public static final int leftAzimuthID = 18;
    public static final int rightAzimuthID = 14;

    public static final int leftDriveID = 17;
    public static final int rightDriveID = 13;

    public static final int leftAbsoluteEncoderID = 3;
    public static final int rightAbsoluteEncoderID = 0;

    public static final int pigeonID = 25;

    public static final boolean leftAzimuthInvert = false;
    public static final boolean rightAzimuthInvert = false;
    public static final boolean leftDriveInvert = false;
    public static final boolean rightDriveInvert = true;

    public static final Translation2d robotCenter = new Translation2d(0, 0); // serves as "center of robot for calculations; robot will turn about this point
    public static final Translation2d leftPodPosition = new Translation2d(-Units.inchesToMeters(5.5), -Units.inchesToMeters(5.5)); // units in meters
    public static final Translation2d rightPodPosition = new Translation2d(Units.inchesToMeters(5.5), Units.inchesToMeters(5.5));
    public static final SwerveDriveKinematics drivetrainKinematics = new SwerveDriveKinematics(robotCenter.minus(leftPodPosition), robotCenter.minus(rightPodPosition));

    public static final double leftAbsoluteEncoderOffset = 0.141906477055928;//absolute encoder reading at position
    public static final double rightAbsoluteEncoderOffset = 3.161239450680035;// gears facing inwards: fwd/bck TODO: less janky alignment

    public static final double azimuthRadiansPerMotorRotation = 2*Math.PI*15/33;
    public static final double driveMetersPerMotorRotation = Units.inchesToMeters(2) * Math.PI * 33 / 45 / 15/13;
    public static final double absoluteRadiansPerEncoderRotation = 2*Math.PI;
    

    

}
