// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LimeLight;
import frc.robot.subsystems.TurdSwerve;

public class TurdFollowAprilTag extends Command {
  private final PIDController pidFollowController;
  private final PIDController pidAngleController;
  private final TurdSwerve swerve;
  private final LimeLight ll;

  private static final double kP_follow = 0.005;
  private static final double kI_follow = 0.00;
  private static final double kD_follow = 0.00;

  private static final double kP_angle = 0.005;
  private static final double kI_angle = 0.00;
  private static final double kD_angle = 0.00;

  public TurdFollowAprilTag(TurdSwerve swerve, LimeLight ll) {
    this.swerve = swerve;
    this.ll = ll;
    this.pidFollowController = new PIDController(kP_follow, kI_follow, kD_follow);
    this.pidAngleController = new PIDController(kP_angle, kI_angle, kD_angle);
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speedOmega = 0.0;
    double speedFollow = 0.0;
    double speedX = 0.0;
    double speedY = 0.0;

    double tX = ll.getTx();
    double tY = ll.getTy();

    double gyro = swerve.getGyro().getRadians();

    if (ll.hasTarget()) {
      speedOmega = pidAngleController.calculate(tX, 0.0);
      speedFollow = pidFollowController.calculate(tY, 0.0);

      speedX = Math.sin(gyro) * speedFollow;
      speedY = -Math.cos(gyro) * speedFollow;
      
    } else {
      speedOmega = 0.0;
      speedX = 0.0;
      speedY = 0.0;
      end(true);
    }
    ChassisSpeeds corrections = new ChassisSpeeds(speedX,speedY, speedOmega);
    swerve.setRobotSpeeds(corrections);
    SmartDashboard.putNumber("Corr. SpeedX", speedX);
    SmartDashboard.putNumber("Corr. SpeedY", speedY);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ChassisSpeeds end = new ChassisSpeeds(0,0,0);
    swerve.setRobotSpeeds(end);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
