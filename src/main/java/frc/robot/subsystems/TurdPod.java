// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TurdPod extends SubsystemBase {

  private final CANSparkMax azimuth;
  private final CANSparkMax drive;
  private final PWM encoder;

  public TurdPod(int azimuthID, int driveID, int encoderID) {
    azimuth = new CANSparkMax(azimuthID, MotorType.kBrushless);
    drive = new CANSparkMax(driveID, MotorType.kBrushless);
    encoder = new PWM(encoderID);
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
