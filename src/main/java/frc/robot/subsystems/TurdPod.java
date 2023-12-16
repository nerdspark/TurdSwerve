// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class TurdPod extends SubsystemBase {

  private final CANSparkMax azimuth;
  private final CANSparkMax drive;
  private final PWM absoluteEncoder;

  private final Encoder azimuthEncoder;
  private final Encoder driveEncoder;

  private double absoluteEncoderOffset;

  public TurdPod(int azimuthID, int driveID, int absoluteEncoderID, boolean azimuthInvert, boolean driveInvert, double absoluteEncoderOffset) {
    azimuth = new CANSparkMax(azimuthID, MotorType.kBrushless);
    drive = new CANSparkMax(driveID, MotorType.kBrushless);
    absoluteEncoder = new PWM(absoluteEncoderID);

    azimuthEncoder = new Encoder(null, null);
    driveEncoder = new Encoder(null, null);
    
    azimuth.setInverted(azimuthInvert);
    drive.setInverted(driveInvert);

    driveEncoder.setDistancePerPulse(Constants.azimuthRadiansPerPulse);
    azimuthEncoder.setDistancePerPulse(Constants.driveMetersPerPulse);

    this.absoluteEncoderOffset = absoluteEncoderOffset;

    azimuth.setSmartCurrentLimit(Constants.azimuthAmpLimit);
    drive.setSmartCurrentLimit(Constants.driveAmpLimit);

    azimuth.setIdleMode(Constants.azimuthMode);
    drive.setIdleMode(Constants.driveMode);
  }

  public void calibratePod() {
    driveEncoder.reset();
    azimuthEncoder.initSendable(null);
  }

  public double getAzimuthAngle() {
    return azimuthEncoder.getDistance();
  }

  public double getDrivePosition() {
    return driveEncoder.getDistance();
  }

  public double getDriveVelocity() {
    return driveEncoder.getRate();
  }

  public double getAbsoluteEncoder() {
    return absoluteEncoder.getPosition() - absoluteEncoderOffset;
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
