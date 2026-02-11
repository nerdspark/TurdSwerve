// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.AnalogEncoder;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.ClosedLoopConfigAccessor;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkMaxConfigAccessor;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;

import frc.robot.constants.Constants;
import frc.robot.constants.RobotMap;

public class TurdPod extends SubsystemBase {
  private final SparkMax azimuth;
  private final SparkMax drive;
  private final AnalogEncoder absoluteEncoder;

  private final ClosedLoopConfigAccessor azimuthPID;
  private final RelativeEncoder azimuthEncoder;
  private final RelativeEncoder driveEncoder;

  private double azimuthDriveSpeedMultiplier;
  private double speed = 0;
  private double absoluteEncoderOffset;
  private double driveSpeedToPower = Constants.driveSpeedToPower;

  private final SparkMaxConfig azimuthConfig;
  private final SparkMaxConfig driveConfig;

  public TurdPod(int azimuthID, int driveID, int absoluteEncoderID, boolean azimuthInvert, boolean driveInvert, double absoluteEncoderOffset) {
    azimuth = new SparkMax(azimuthID, MotorType.kBrushless);
    drive = new SparkMax(driveID, MotorType.kBrushless);
    absoluteEncoder = new AnalogEncoder(absoluteEncoderID);

    azimuthEncoder = azimuth.getEncoder();
    driveEncoder = drive.getEncoder();

    // Azimuth Configuration
    this.azimuthConfig = new SparkMaxConfig();
    azimuthConfig.encoder.positionConversionFactor(RobotMap.azimuthRadiansPerMotorRotation);
    azimuthConfig.smartCurrentLimit(Constants.azimuthAmpLimit);
    azimuthConfig.idleMode(Constants.azimuthMode);
    azimuthConfig.inverted(azimuthInvert);
    azimuth.configure(this.azimuthConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    azimuthPID = azimuth.configAccessor.closedLoop;

    // Drive Configuration
    this.driveConfig = new SparkMaxConfig();
    driveConfig.encoder.positionConversionFactor(RobotMap.driveMetersPerMotorRotation);
    driveConfig.openLoopRampRate(Constants.driveMotorRampRate);
    driveConfig.smartCurrentLimit(Constants.driveAmpLimit);
    driveConfig.idleMode(Constants.driveMode);
    driveConfig.inverted(driveInvert);    
    drive.configure(this.driveConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    
    // absoluteEncoder.setPositionOffset(absoluteEncoderOffset);
    this.absoluteEncoderOffset = absoluteEncoderOffset;

    resetPod();
  }
  
  public void setAmpLimit(int ampLimit) {
    driveConfig.smartCurrentLimit(ampLimit);
    drive.configure(this.driveConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
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
    double convertedPosition = absoluteEncoder.get() * RobotMap.absoluteRadiansPerEncoderRotation;
    absoluteEncoderOffset = (convertedPosition * 2*Math.PI);
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
    if (P != azimuthPID.getP()) {azimuthConfig.closedLoop.p(P);}
    if (I != azimuthPID.getI()) {azimuthConfig.closedLoop.i(I);}
    if (D != azimuthPID.getD()) {azimuthConfig.closedLoop.d(D);}
    if (IZone != azimuthPID.getIZone()) {azimuthConfig.closedLoop.iZone(IZone);}
    if (outputRange != azimuthPID.getMaxOutput()) {
      azimuthConfig.closedLoop.minOutput(-outputRange);
      azimuthConfig.closedLoop.maxOutput(-outputRange);
    }
    azimuthConfig.closedLoop.positionWrappingMaxInput(Math.PI);
    azimuthConfig.closedLoop.positionWrappingMinInput(-Math.PI);
    azimuthConfig.closedLoop.positionWrappingEnabled(true);
    // azimuthPID.setSmartMotionAllowedClosedLoopError(0, 0);
    azimuthConfig.closedLoopRampRate(0.35);
    azimuthDriveSpeedMultiplier = ADMult;

    azimuth.configure(azimuthConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  public SwerveModulePosition getPodPosition() {
    return new SwerveModulePosition(driveEncoder.getPosition(), new Rotation2d(azimuthEncoder.getPosition()));
  }

  public void setPodState(SwerveModuleState state) {
    state = SwerveModuleState.optimize(state, new Rotation2d(azimuthEncoder.getPosition())); // does not account for rotations between 180 and 360?
    azimuth.getClosedLoopController().setSetpoint(state.angle.getRadians(), ControlType.kPosition);
    speed = Math.abs(state.speedMetersPerSecond) < .01 ? 0 : state.speedMetersPerSecond * driveSpeedToPower;
    SmartDashboard.putNumber("state.angle.getRadians()", state.angle.getRadians());

    double error = (state.angle.getRadians() - azimuthEncoder.getPosition()) % (2*Math.PI);
      error = error > Math.PI ? error - 2*Math.PI : error;
      error = error < -Math.PI ? error + 2*Math.PI : error;
      error *= 180 / Math.PI;
      SmartDashboard.putNumber("error azimuth " + azimuth.getDeviceId(), error);
  }

  public double getAbsoluteEncoder() {
    double convertedPosition = absoluteEncoder.get() * RobotMap.absoluteRadiansPerEncoderRotation;
    return (convertedPosition * 2*Math.PI) - absoluteEncoderOffset;
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
