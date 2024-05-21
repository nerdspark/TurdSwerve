// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.REVTurdSwerve;

public class TurdDrive extends Command {
  
  REVTurdSwerve swerve;
  Supplier<Translation2d> joystickRight, joystickLeft;
  Supplier<Integer> DPAD;
  Supplier<Double> robotMaxSpeed;
  Rotation2d rotation = new Rotation2d();

  public TurdDrive(REVTurdSwerve swerve, Supplier<Translation2d> joystickLeft, Supplier<Translation2d> joystickRight, Supplier<Integer> DPAD, Supplier<Double> robotMaxSpeed) {
    this.swerve = swerve;
    this.joystickRight = joystickRight;
    this.joystickLeft = joystickLeft;
    this.DPAD = DPAD;
    this.robotMaxSpeed = robotMaxSpeed;
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    swerve.resetGyro();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (DPAD.get() != -1) {
      swerve.targetAngle = -Units.degreesToRadians(DPAD.get());
    }

    if (robotMaxSpeed.get() > 0.95) {
      swerve.setAmpLimit(Constants.driveTopAmpLimit);
    } else {
      swerve.setAmpLimit(Constants.driveAmpLimit);
    }
    double speedX = MathUtil.applyDeadband(-joystickRight.get().getX(), 0.05) * robotMaxSpeed.get();
    double speedY = MathUtil.applyDeadband(joystickRight.get().getY(), 0.05) * robotMaxSpeed.get();
    // double speedX = deadband ? 0 : 3.0 * Math.abs(joystickRight.get().getX()) * -joystickRight.get().getX();
    // double speedY = deadband ? 0 : 3.0 * Math.abs(joystickRight.get().getY()) * joystickRight.get().getY();
    double speedOmega = Math.abs(joystickLeft.get().getX()) > 0.07 ? -joystickLeft.get().getX() * Math.abs(joystickLeft.get().getX()) : 0;
    ChassisSpeeds speeds = new ChassisSpeeds(speedX, speedY, speedOmega);
    swerve.setRobotSpeeds(speeds);
    
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
