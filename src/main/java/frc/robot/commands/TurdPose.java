// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
//import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.LimeLight;
import frc.robot.subsystems.TurdSwerve;

public class TurdPose extends Command {
  private final PIDController pidFollowController;
  private final PIDController pidAngleController;
  private final TurdSwerve swerve;
  private final LimeLight ll;
  private static final double kP_follow = 0.1;
  private static final double kI_follow = 0.00;
  private static final double kD_follow = 0.00;

  private static final double kP_angle = 0.005;
  private static final double kI_angle = 0.00;
  private static final double kD_angle = 0.00;

  double speedOmega = 0.0;
  double speedX = 0.0;
  double speedY = 0.0;
      
  final double[][] tagFieldCoords = {
      {-2, 2},
      {0, 2},
      {2, 2},
      {2.41, 0}
    };

  /** Creates a new TurdPose. */
  public TurdPose(TurdSwerve swerve, LimeLight ll) {
    this.swerve = swerve;
    this.ll = ll;
    this.pidFollowController = new PIDController(kP_follow, kI_follow, kD_follow);
    this.pidAngleController = new PIDController(kP_angle, kI_angle, kD_angle);
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ll.setPipelineNumber(0);
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double[] relbotPose = ll.getRelBotPose();
    double tX = relbotPose[0];
    double tz = relbotPose[2];
    long tID = ll.getID();
    double yaw = relbotPose[4];
    double gyro = swerve.getGyro().getDegrees();
    
    if (tID > 0) {
      double[] currentTag = tagFieldCoords[(int) tID - 1];
      
      double robotX = currentTag[0] - tX;
      double robotY = currentTag[1] - tz;

      speedOmega = pidAngleController.calculate(gyro, 0.0);

      double distance = Math.sqrt(robotX * robotX + robotY * robotY);
      double poseToRobotTheta = Math.atan2(robotX, robotY);

      speedX = -pidFollowController.calculate(distance, 0.0) * Math.sin(poseToRobotTheta);
      speedY = -pidFollowController.calculate(distance, 0.0) * Math.cos(poseToRobotTheta);

      SmartDashboard.putNumber("Yaw", yaw);
      SmartDashboard.putNumber("tID", tID);
      SmartDashboard.putNumber("robotX", robotX);
      SmartDashboard.putNumber("robotY", robotY);
      SmartDashboard.putNumber("correctionX", speedX);
      SmartDashboard.putNumber("correctionY", speedY);
      SmartDashboard.putNumber("correctionOmega", speedOmega);
      SmartDashboard.putNumber("distance", distance);
      SmartDashboard.putNumber("poseToRobotTheta", poseToRobotTheta * 180 / Math.PI);
    }

    ChassisSpeeds corrections = new ChassisSpeeds(speedX, speedY, speedOmega);
    swerve.setRobotSpeeds(corrections);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
