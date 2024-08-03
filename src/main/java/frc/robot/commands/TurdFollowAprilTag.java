// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LimeLight;
import frc.robot.subsystems.TurdSwerve;

public class TurdFollowAprilTag extends Command {
  private final TurdSwerve swerve;
  private final LimeLight ll;
  private final double kP_Aim = 0.05;
  private final double min_aim_cmd = 0.01;
  private double tX = 0, tY = 0;
  public TurdFollowAprilTag(TurdSwerve swerve, LimeLight ll) {
    this.swerve = swerve;
    this.ll = ll;
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
    double tX = ll.getTx();
    if (tX != 0.00) {
      if (tX < -9) {
        speedOmega = 0.1;
      } else if (tX > 9) {
        speedOmega = -0.1;
      } else {
        end(false);
      }
    } else {
      end(false);
    }
    ChassisSpeeds corrections = new ChassisSpeeds(0,0, speedOmega);
    swerve.setRobotSpeeds(corrections);
    // tY = ll.getTy();
    // double correctionXSpeed = 0, correctionYSpeed = 0;
    // if (tX > 0.75) {
    //   correctionXSpeed = -tX * kP_Aim - min_aim_cmd;
    // }
    // if (tX < -0.75) {
    //   correctionXSpeed = tX * kP_Aim + min_aim_cmd;
    // }
    // if (tX >= -0.75 && tX <= 0.75) {
    //   correctionXSpeed = 0;
    // }
    // if (tY < 0) {
    //   correctionYSpeed = tY * kP_Aim + min_aim_cmd;
    // }
    // if (tY >= 0) {
    //   correctionYSpeed = 0;
    // }
    // SmartDashboard.putNumber("CorrectX", correctionXSpeed);
    // SmartDashboard.putNumber("CorrectY", correctionYSpeed);
    // ChassisSpeeds correctionSpeed = new ChassisSpeeds(correctionXSpeed, correctionYSpeed, 0);
    // swerve.setRobotSpeeds(correctionSpeed);
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
