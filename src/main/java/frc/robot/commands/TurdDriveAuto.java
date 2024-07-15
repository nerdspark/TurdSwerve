// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.TurdSwerve;

public class TurdDriveAuto extends Command {
  TurdSwerve swerve;
  double m_SpeedX, m_SpeedY;
  ChassisSpeeds speeds;
  /** Creates a new TurdDriveAuto. */
  public TurdDriveAuto(TurdSwerve swerve, double speed, double theta) {
    this.swerve = swerve;
    double theta_deg = theta * Math.PI / 180;
    m_SpeedX = speed * Math.cos(theta_deg);
    m_SpeedY = speed * Math.sin(theta_deg);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Constants.aPressed = true;
    speeds = new ChassisSpeeds(m_SpeedX, m_SpeedY, 0);
    swerve.setRobotSpeeds(speeds);
    SmartDashboard.putBoolean("A Pressed", Constants.aPressed);
    SmartDashboard.putNumber("X Speed", m_SpeedX);
    SmartDashboard.putNumber("Y Speed", m_SpeedY);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) { }


  // Returns true when the command should end.
}
