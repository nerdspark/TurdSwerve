// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.RobotMap;

import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.wpilibj.DataLogManager;

public class TurdSwerve extends SubsystemBase {
  private final SlewRateLimiter xLimiter = new SlewRateLimiter(0.75);
  private final SlewRateLimiter yLimiter = new SlewRateLimiter(0.75);
  private final Pigeon2 gyro = new Pigeon2(RobotMap.pigeonID);
  private final TurdPod leftPod = new TurdPod(RobotMap.leftAzimuthID, RobotMap.leftDriveID, RobotMap.leftAbsoluteEncoderID, RobotMap.leftAzimuthInvert, RobotMap.leftDriveInvert, RobotMap.leftAbsoluteEncoderOffset);
  private final TurdPod rightPod = new TurdPod(RobotMap.rightAzimuthID, RobotMap.rightDriveID, RobotMap.rightAbsoluteEncoderID, RobotMap.rightAzimuthInvert, RobotMap.rightDriveInvert, RobotMap.rightAbsoluteEncoderOffset);
  private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(RobotMap.drivetrainKinematics,
          new Rotation2d(0), new SwerveModulePosition[] {
              leftPod.getPodPosition(),
              rightPod.getPodPosition()
          });

  private ShuffleboardTab tab = Shuffleboard.getTab("PID");
  private GenericEntry azimuthP = tab.add("azimuth P", Constants.azimuthkP).getEntry();
  private GenericEntry azimuthI = tab.add("azimuth I", Constants.azimuthkI).getEntry();
  private GenericEntry azimuthD = tab.add("azimuth D", Constants.azimuthkD).getEntry();
  private GenericEntry azimuthIzone = tab.add("azimuth IZone", Constants.azimuthkD).getEntry();
  private GenericEntry ADMult = tab.add("azimuth-drive speed multiplier", Constants.azimuthDriveSpeedMultiplier).getEntry();

  private PIDController GyroPID = new PIDController(Constants.gyroP, Constants.gyroI, Constants.gyroD);
  public double targetAngle = 0;
  private double odoAngleOffset = Math.PI * 0.0;

  private Rotation2d gyroResetAngle = new Rotation2d();
  
  
  private final Field2d field2d = new Field2d();

  BooleanLogEntry drivingLogLeft;
  DoubleLogEntry amperageLogAzimuthLeft;
  DoubleLogEntry amperageLogDriveLeft;
  BooleanLogEntry drivingLogRight;
  DoubleLogEntry amperageLogAzimuthRight;
  DoubleLogEntry amperageLogDriveRight;
  DoubleLogEntry outputLogAzimuthLeft;
  DoubleLogEntry outputLogDriveLeft;
  DoubleLogEntry outputLogAzimuthRight;
  DoubleLogEntry outputLogDriveRight;

  public TurdSwerve() {
    DataLogManager.start();
    
    DataLog log = DataLogManager.getLog();
    drivingLogLeft = new BooleanLogEntry(log, "Driving? (Left): ");
    amperageLogAzimuthLeft = new DoubleLogEntry(log, "Amperage (Azimuth Left): ");
    amperageLogDriveLeft = new DoubleLogEntry(log, "Amperage (Drive Left): ");
    drivingLogRight = new BooleanLogEntry(log, "Driving? (Right): ");
    amperageLogAzimuthRight = new DoubleLogEntry(log, "Amperage (Azimuth Right): ");
    amperageLogDriveRight = new DoubleLogEntry(log, "Amperage (Drive Right): ");
    outputLogAzimuthLeft = new DoubleLogEntry(log, "Output (Azimuth Left): ");
    outputLogDriveLeft = new DoubleLogEntry(log, "Output (Drive Left): ");
    outputLogAzimuthRight = new DoubleLogEntry(log, "Output (Azimuth Right): ");
    outputLogDriveRight = new DoubleLogEntry(log, "Output (Drive Right): ");

    GyroPID.enableContinuousInput(0.0, 2*Math.PI);
    // gyro.configAllSettings(new Pigeon2Configuration());
  }

  public void setAmpLimit(int ampLimit) {
    leftPod.setAmpLimit(ampLimit);
    rightPod.setAmpLimit(ampLimit);
  }

  // public void setDriveSpeedtoPower(double driveSpeedToPower) {
  //   leftPod.setDriveSpeedtoPower(driveSpeedToPower);
  //   rightPod.setDriveSpeedtoPower(driveSpeedToPower);
  // }

  public void resetOdometry(Pose2d pose) {
    odoAngleOffset = DriverStation.getAlliance().get() == Alliance.Red ? Math.PI * 0.5 : Math.PI * 1.5;
    odometer.resetPosition(new Rotation2d(odoAngleOffset), new SwerveModulePosition[] {leftPod.getPodPosition(), rightPod.getPodPosition()}, pose);
  }

  public void resetPods() {
    resetGyro();
    leftPod.resetPod();
    rightPod.resetPod();
    leftPod.setPID(azimuthP.getDouble(Constants.azimuthkP), azimuthI.getDouble(Constants.azimuthkI), azimuthD.getDouble(Constants.azimuthkD), azimuthIzone.getDouble(Constants.azimuthkIz), Constants.azimuthMaxOutput, ADMult.getDouble(Constants.azimuthDriveSpeedMultiplier));
    rightPod.setPID(azimuthP.getDouble(Constants.azimuthkP), azimuthI.getDouble(Constants.azimuthkI), azimuthD.getDouble(Constants.azimuthkD), azimuthIzone.getDouble(Constants.azimuthkIz), Constants.azimuthMaxOutput, ADMult.getDouble(Constants.azimuthDriveSpeedMultiplier));
    resetOdometry(new Pose2d(new Translation2d(8.0, 4.2), new Rotation2d()));
  }

  public void resetZero() {
    leftPod.resetZero();
    rightPod.resetZero();
  }

  public void revertZero() {
    leftPod.revertZero();
    rightPod.revertZero();
  }

  public void stop() {
    leftPod.stop();
    rightPod.stop();
  }

  public Rotation2d getGyro() {
    return new Rotation2d(-gyro.getAngle()*Math.PI/180).minus(gyroResetAngle);
  }

  public void resetGyro() {
    gyroResetAngle = getGyro().plus(gyroResetAngle);
    targetAngle = 0;
  }

  // public void setLeftPod(SwerveModuleState state) {
  //   leftPod.setPodState(state);
  // }

  public void setRobotSpeeds(ChassisSpeeds chassisSpeeds) {
    boolean manualTurn = true;//Math.abs(chassisSpeeds.omegaRadiansPerSecond) > 0.1;
    // chassisSpeeds = chassisSpeeds.times(robotMaxSpeed);
    chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xLimiter.calculate(chassisSpeeds.vxMetersPerSecond), yLimiter.calculate(chassisSpeeds.vyMetersPerSecond), manualTurn ? chassisSpeeds.omegaRadiansPerSecond * 3.0 : GyroPID.calculate(getGyro().getRadians(), targetAngle), getGyro());
    SwerveModuleState[] states = RobotMap.drivetrainKinematics.toSwerveModuleStates(chassisSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.podMaxSpeed);
    if (manualTurn) {
    targetAngle = getGyro().getRadians() + (chassisSpeeds.omegaRadiansPerSecond / 2.0);
    }

    leftPod.setPodState(states[0]);
    rightPod.setPodState(states[1]);
  }

  public void telemetry() {
    // double leftSpeed = leftPod.getSpeed(null);
    // double rightSpeed = rightPod.getSpeed(null);
    // if (leftSpeed != 0) {
    //   drivingLogLeft.append(true);
    //   SmartDashboard.putBoolean("Driving? (Left): ", true);
    // } else {
    //   drivingLogLeft.append(false);
    //   SmartDashboard.putBoolean("Driving? (Left): ", false);
    // }
    // if (rightSpeed != 0) {
    //   drivingLogRight.append(true);
    //   SmartDashboard.putBoolean("Driving? (Right): ", true);
    // } else {
    //   drivingLogRight.append(false);
    //   SmartDashboard.putBoolean("Driving? (Right): ", false);
    // }
    
    double OutputAzimuthLeft = leftPod.getAzimuthOutput();
    double OutputDriveLeft = leftPod.getDriveOutput();
    double OutputAzimuthRight = rightPod.getAzimuthOutput();
    double OutputDriveRight = rightPod.getDriveOutput();
    double AmperageAzimuthLeft = leftPod.getAzimuthAmp();
    double AmperageDriveLeft = leftPod.getDriveAmp();
    double AmperageAzimuthRight = rightPod.getAzimuthAmp();
    double AmperageDriveRight = rightPod.getDriveAmp();
    amperageLogAzimuthLeft.append(AmperageAzimuthLeft);
    amperageLogDriveLeft.append(AmperageDriveLeft);
    amperageLogAzimuthRight.append(AmperageAzimuthRight);
    amperageLogDriveRight.append(AmperageDriveRight);
    outputLogAzimuthLeft.append(OutputAzimuthLeft);
    outputLogDriveLeft.append(OutputDriveLeft);
    outputLogAzimuthRight.append(OutputAzimuthRight);
    outputLogDriveRight.append(OutputDriveRight);
    SmartDashboard.putNumber("Amperage (Azimuth Left)", AmperageAzimuthLeft);
    SmartDashboard.putNumber("Amperage (Drive Left)", AmperageDriveLeft);
    SmartDashboard.putNumber("Amperage (Azimuth Right)", AmperageAzimuthRight);
    SmartDashboard.putNumber("Amperage (Drive Right)", AmperageDriveRight);
    SmartDashboard.putNumber("Output (Azimuth Left): ", OutputAzimuthLeft);
    SmartDashboard.putNumber("Output (Drive Left): ", OutputDriveLeft);
    SmartDashboard.putNumber("Output (Azimuth Right): ", OutputAzimuthRight);
    SmartDashboard.putNumber("Output (Drive Right): ", OutputDriveRight);
  }
  @Override
  public void periodic() {
    odometer.update(getGyro(), new SwerveModulePosition[] {leftPod.getPodPosition(), rightPod.getPodPosition()});
    SmartDashboard.putNumber("pigeon", getGyro().getDegrees());
    field2d.setRobotPose(odometer.getPoseMeters().transformBy(new Transform2d(new Translation2d(), new Rotation2d(odoAngleOffset + Math.PI))));
    telemetry();
  }
  
  private String getFomattedPose() {
    var pose = odometer.getPoseMeters();
    return String.format(
            "(%.3f, %.3f) %.2f degrees",
            pose.getX(), pose.getY(), pose.getRotation().plus(new Rotation2d(odoAngleOffset)).getDegrees());
  }
  
  public void addDashboardWidgets(ShuffleboardTab tab) {
    tab.add("Field", field2d).withPosition(0, 0).withSize(6, 4);
    tab.addString("Pose", this::getFomattedPose).withPosition(6, 2).withSize(2, 1);
  }

  public double[] getDriveAmps() {
    return new double[] {leftPod.getDriveAmp(), rightPod.getDriveAmp()};
  }
}
