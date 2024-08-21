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
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.constants.Constants;
import frc.robot.constants.RobotMap;

public class TurdSwerve extends SubsystemBase {
  private final SlewRateLimiter xLimiter = new SlewRateLimiter(0.75);
  private final SlewRateLimiter yLimiter = new SlewRateLimiter(0.75);
  private final Pigeon2 gyro = new Pigeon2(RobotMap.pigeonID);
  private final TurdPod leftPod = new TurdPod(RobotMap.leftAzimuthID, RobotMap.leftDriveID, RobotMap.leftAbsoluteEncoderID, RobotMap.leftAzimuthInvert, RobotMap.leftDriveInvert, RobotMap.leftAbsoluteEncoderOffset);
  private final TurdPod rightPod = new TurdPod(RobotMap.rightAzimuthID, RobotMap.rightDriveID, RobotMap.rightAbsoluteEncoderID, RobotMap.rightAzimuthInvert, RobotMap.rightDriveInvert, RobotMap.rightAbsoluteEncoderOffset);
  public final SwerveDriveOdometry odometer = new SwerveDriveOdometry(RobotMap.drivetrainKinematics,
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
  private GenericEntry driveP = tab.add("drive P", Constants.drivekP).getEntry();
  private GenericEntry driveI = tab.add("drive I", Constants.drivekI).getEntry();
  private GenericEntry driveD = tab.add("drive D", Constants.drivekD).getEntry();
  private GenericEntry driveIzone = tab.add("drive IZone", Constants.drivekD).getEntry();
  private GenericEntry DDMult = tab.add("drive-drive speed multiplier", Constants.driveDriveSpeedMultiplier).getEntry();
  private PIDController GyroPID = new PIDController(Constants.gyroP, Constants.gyroI, Constants.gyroD);
  private PIDController leftDrivePID = new PIDController(Constants.drivekP, Constants.drivekI, Constants.drivekD);
  private PIDController rightDrivePID = new PIDController(Constants.drivekP, Constants.drivekI, Constants.drivekD);
  public double targetAngle = 0;
  private double odoAngleOffset = Math.PI * 0.0;
  private double targetDistance = 0;

  private Rotation2d gyroResetAngle = new Rotation2d();
  
  
  private final Field2d field2d = new Field2d();
  
  private boolean useDrivePID = false;

  public TurdSwerve() {
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
    leftPod.setPID(azimuthP.getDouble(Constants.azimuthkP), azimuthI.getDouble(Constants.azimuthkI), azimuthD.getDouble(Constants.azimuthkD), azimuthIzone.getDouble(Constants.azimuthkIz), driveP.getDouble(Constants.drivekP), driveI.getDouble(Constants.drivekI), driveD.getDouble(Constants.drivekD), driveIzone.getDouble(Constants.drivekIz), Constants.azimuthMaxOutput, ADMult.getDouble(Constants.azimuthDriveSpeedMultiplier), DDMult.getDouble(Constants.driveDriveSpeedMultiplier));
    rightPod.setPID(azimuthP.getDouble(Constants.azimuthkP), azimuthI.getDouble(Constants.azimuthkI), azimuthD.getDouble(Constants.azimuthkD), azimuthIzone.getDouble(Constants.azimuthkIz), driveP.getDouble(Constants.drivekP), driveI.getDouble(Constants.drivekI), driveD.getDouble(Constants.drivekD), driveIzone.getDouble(Constants.drivekIz), Constants.azimuthMaxOutput, ADMult.getDouble(Constants.azimuthDriveSpeedMultiplier), DDMult.getDouble(Constants.driveDriveSpeedMultiplier));
    resetOdometry(new Pose2d(new Translation2d(0.0, 0.0), new Rotation2d()));
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

  public void cease() {
    leftPod.drive.set(0);
    leftPod.azimuth.set(0);
    rightPod.azimuth.set(0);
    rightPod.drive.set(0);
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
  // public void drive(double meters) {
  //   // double driveSpeed = drivePID.calculate(meters);
  //   // SmartDashboard.putNumber("Drive Speed:", driveSpeed);
  //   // ChassisSpeeds speeds = new ChassisSpeeds(0, -0.25, 0);
  //   // ChassisSpeeds stop = new ChassisSpeeds(0,0,0);
  //   // double time = meters / 0.25;
  //   // setRobotSpeeds(speeds);
  //   // new WaitCommand(time);
  //   // setRobotSpeeds(stop);
  //   useDrivePID = true;
  //   leftDrivePID.setSetpoint(meters);
  //   rightDrivePID.setSetpoint(meters);
  //   targetDistance = meters;
  // }

  public void turn(double radians) {
    leftPod.azimuth.set(GyroPID.calculate(radians));
    rightPod.azimuth.set(GyroPID.calculate(radians));
  }

  @Override
  public void periodic() {
    odometer.update(getGyro(), new SwerveModulePosition[] {leftPod.getPodPosition(), rightPod.getPodPosition()});
    SmartDashboard.putNumber("pigeon", getGyro().getDegrees());
    SmartDashboard.putNumber("lenc pose", leftPod.driveEncoder.getPosition());
    SmartDashboard.putNumber("renc pose", rightPod.driveEncoder.getPosition());
    SmartDashboard.putBoolean("drivePID?", useDrivePID);
    SmartDashboard.putNumber("target", targetDistance);
    field2d.setRobotPose(odometer.getPoseMeters().transformBy(new Transform2d(new Translation2d(), new Rotation2d(odoAngleOffset + Math.PI))));
    // if (useDrivePID) {
    //   double leftSpeed = leftDrivePID.calculate(leftPod.driveEncoder.getPosition());
    //   double rightSpeed = rightDrivePID.calculate(rightPod.driveEncoder.getPosition());
    //   double averageSpeed = (leftSpeed + rightSpeed) / 2;
    //   ChassisSpeeds speeds = new ChassisSpeeds(averageSpeed,0,0);
    //   setRobotSpeeds(speeds);     
    // }
    // if (leftDrivePID.atSetpoint()) {
    //   targetDistance = 0;
    //   useDrivePID = false;
    //   cease();
    // }
    
  }
  
  public String getFomattedPose() {
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