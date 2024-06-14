// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.RobotMap;

public class TurdPod extends SubsystemBase {

  private final CANSparkMax azimuth;
  private final CANSparkMax drive;
  private final AnalogEncoder absoluteEncoder;

  private final RelativeEncoder azimuthEncoder;
  private final RelativeEncoder driveEncoder;
  private final SparkPIDController azimuthPID;

  private double azimuthDriveSpeedMultiplier;
  private double speed = 0;
  private double absoluteEncoderOffset;
  private double driveSpeedToPower = Constants.driveSpeedToPower;

  public TurdPod(int azimuthID, int driveID, int absoluteEncoderID, boolean azimuthInvert, boolean driveInvert, double absoluteEncoderOffset) {

    azimuth = new CANSparkMax(azimuthID, MotorType.kBrushless);
    drive = new CANSparkMax(driveID, MotorType.kBrushless);
    absoluteEncoder = new AnalogEncoder(absoluteEncoderID);

    azimuthEncoder = azimuth.getEncoder();
    driveEncoder = drive.getEncoder();
    
    azimuth.setInverted(azimuthInvert);
    drive.setInverted(driveInvert);

    driveEncoder.setPositionConversionFactor(RobotMap.driveMetersPerMotorRotation);
    azimuthEncoder.setPositionConversionFactor(RobotMap.azimuthRadiansPerMotorRotation);
    absoluteEncoder.setDistancePerRotation(RobotMap.absoluteRadiansPerEncoderRotation);

    // absoluteEncoder.setPositionOffset(absoluteEncoderOffset);
    this.absoluteEncoderOffset = absoluteEncoderOffset;

    azimuth.setSmartCurrentLimit(Constants.azimuthAmpLimit);
    drive.setSmartCurrentLimit(Constants.driveAmpLimit);

    azimuth.setIdleMode(Constants.azimuthMode);
    drive.setIdleMode(Constants.driveMode);

    drive.setOpenLoopRampRate(Constants.driveMotorRampRate);

    azimuthPID = azimuth.getPIDController();

    resetPod();
  }
  
  public void setAmpLimit(int ampLimit) {
    drive.setSmartCurrentLimit(ampLimit);
  } 

  // public void setDriveSpeedtoPower(double driveSpeedToPower) {
  //   this.driveSpeedToPower = driveSpeedToPower;
  // }

  public void resetPod() {
    driveEncoder.setPosition(0);
    azimuthEncoder.setPosition(getAbsoluteEncoder());
  }

  public String getPod() {
    return azimuth.getDeviceId() == RobotMap.leftAzimuthID ? "Left" : "Right";
  }

  public void resetZero() {
    absoluteEncoderOffset = (absoluteEncoder.getAbsolutePosition() * 2*Math.PI);
    SmartDashboard.putNumber((getPod() + " Encoder Offset"), absoluteEncoderOffset);
    resetPod();
  }

  public void revertZero() {
    absoluteEncoderOffset = azimuth.getDeviceId() == RobotMap.leftAzimuthID ? RobotMap.leftAbsoluteEncoderOffset : RobotMap.rightAbsoluteEncoderOffset;
    SmartDashboard.putNumber((getPod() + " Encoder Offset"), absoluteEncoderOffset);
    resetPod();
  }
  
  public void stop() {
    azimuth.set(0);
    drive.set(0);
  }

  public void setPID(double P, double I, double D, double IZone, double outputRange, double ADMult) {
    if (P != azimuthPID.getP()) {azimuthPID.setP(P);}
    if (I != azimuthPID.getI()) {azimuthPID.setI(I);}
    if (D != azimuthPID.getD()) {azimuthPID.setD(D);}
    if (IZone != azimuthPID.getIZone()) {azimuthPID.setIZone(IZone);}
    if (outputRange != azimuthPID.getOutputMax()) {azimuthPID.setOutputRange(-outputRange, outputRange);}
    azimuthPID.setPositionPIDWrappingMaxInput(Math.PI);
    azimuthPID.setPositionPIDWrappingMinInput(-Math.PI);
    azimuthPID.setPositionPIDWrappingEnabled(true);
    // azimuthPID.setSmartMotionAllowedClosedLoopError(0, 0);
    azimuth.setClosedLoopRampRate(0.35);
    azimuthDriveSpeedMultiplier = ADMult;
  }

  public SwerveModulePosition getPodPosition() {
    return new SwerveModulePosition(driveEncoder.getPosition(), new Rotation2d(azimuthEncoder.getPosition()));
  }

  public void setPodState(SwerveModuleState state) {
    state = SwerveModuleState.optimize(state, new Rotation2d(azimuthEncoder.getPosition())); // does not account for rotations between 180 and 360?
    azimuthPID.setReference(state.angle.getRadians(), ControlType.kPosition); 
    speed = getSpeed(state);
    SmartDashboard.putNumber("state.angle.getRadians()", state.angle.getRadians());

    double error = (state.angle.getRadians() - azimuthEncoder.getPosition()) % (2*Math.PI);
      error = error > Math.PI ? error - 2*Math.PI : error;
      error = error < -Math.PI ? error + 2*Math.PI : error;
      error *= 180 / Math.PI;
      SmartDashboard.putNumber("error azimuth " + azimuth.getDeviceId(), error);
  }
  public double getSpeed(SwerveModuleState state) {
    double speed = Math.abs(state.speedMetersPerSecond) < .01 ? 0 : state.speedMetersPerSecond * driveSpeedToPower;
    return speed;
  }
  public double getAzimuthOutput() {
    return azimuth.getAppliedOutput();
  }
  public double getDriveOutput() {
    return drive.getAppliedOutput();
  }
  

  public double getAbsoluteEncoder() {
    return (absoluteEncoder.getAbsolutePosition() * 2*Math.PI) - absoluteEncoderOffset;
  }

    public double getAzimuthAmp() {
    return azimuth.getOutputCurrent();
  }
  public double getDriveAmp() {
    return drive.getOutputCurrent();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("getabsoluteEncoder() " + absoluteEncoder.getChannel(), getAbsoluteEncoder());
    drive.set(speed + (azimuth.getAppliedOutput() * azimuthDriveSpeedMultiplier));
    SmartDashboard.putNumber("azimuthEncoder.getPosition() " + azimuth.getDeviceId(), azimuthEncoder.getPosition());
    SmartDashboard.putNumber("drive pos " + drive.getDeviceId(), driveEncoder.getPosition());
    SmartDashboard.putNumber("azimuth.getoutputcurrent()" + azimuth.getDeviceId(), azimuth.getOutputCurrent());
    SmartDashboard.putNumber("drive.getoutputcurrent()" + drive.getDeviceId(), drive.getOutputCurrent());
  }

}
