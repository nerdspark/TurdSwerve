// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AnalogInput;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class TurdPod extends SubsystemBase {

  private final CANSparkMax azimuth;
  private final CANSparkMax drive;
  private final AnalogEncoder absoluteEncoder;

  private final RelativeEncoder azimuthEncoder;
  private final RelativeEncoder driveEncoder;

  private final PIDController azimuthPID = new PIDController(Constants.azimuthkP, Constants.azimuthkI, Constants.azimuthkD);

  public TurdPod(int azimuthID, int driveID, int absoluteEncoderID, boolean azimuthInvert, boolean driveInvert, double absoluteEncoderOffset) {
    azimuth = new CANSparkMax(azimuthID, MotorType.kBrushless);
    drive = new CANSparkMax(driveID, MotorType.kBrushless);
    absoluteEncoder = new AnalogEncoder(absoluteEncoderID);

    azimuthEncoder = azimuth.getEncoder();
    driveEncoder = drive.getEncoder();
    
    azimuth.setInverted(azimuthInvert);
    drive.setInverted(driveInvert);

    driveEncoder.setPositionConversionFactor(Constants.driveMetersPerPulse);
    azimuthEncoder.setPositionConversionFactor(Constants.azimuthRadiansPerPulse);
    absoluteEncoder.setDistancePerRotation(Constants.absoluteEncoderRadiansPerRotation);

    absoluteEncoder.setPositionOffset(absoluteEncoderOffset);

    azimuth.setSmartCurrentLimit(Constants.azimuthAmpLimit);
    drive.setSmartCurrentLimit(Constants.driveAmpLimit);

    azimuth.setIdleMode(Constants.azimuthMode);
    drive.setIdleMode(Constants.driveMode);

    resetPod();
  }

  public void resetPod() {
    driveEncoder.setPosition(0);
    azimuthEncoder.setPosition(getAbsoluteEncoder());
  }

  public SwerveModuleState getPodState() {
    return new SwerveModuleState(driveEncoder.getVelocity(), new Rotation2d(azimuthEncoder.getPosition()));
  }

  public void setPodState(SwerveModuleState state) {
    drive.set(state.speedMetersPerSecond);
    azimuth.set(azimuthPID.calculate(state.angle.getRadians()));
  }

  public double getAbsoluteEncoder() {
    return absoluteEncoder.getAbsolutePosition();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("absoluteEncoder " + absoluteEncoder.getChannel(), getAbsoluteEncoder());

  }
}
