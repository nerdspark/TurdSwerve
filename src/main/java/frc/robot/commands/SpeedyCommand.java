// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.TurdSwerve;

public class SpeedyCommand extends Command {
  private TurdSwerve swerve;
  /** Creates a new SpeedyCommand. */
  public SpeedyCommand(TurdSwerve swerve) {
    this.swerve = swerve;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    swerve.setAmpLimit(Constants.driveTopAmpLimit);
    swerve.setDriveSpeedtoPower(Constants.driveTopSpeedToPower);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerve.setAmpLimit(Constants.driveAmpLimit);
    swerve.setDriveSpeedtoPower(Constants.driveSpeedToPower);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}