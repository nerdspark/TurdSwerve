// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TurdPod;
import frc.robot.subsystems.TurdSwerve;
import frc.robot.Constants;

public class TurdDrive extends Command {
  
  TurdSwerve swerve;
  Supplier<Translation2d> joystickRight;
  Supplier<Translation2d> joystickLeft;
  Supplier<Boolean> resetPods;
  Supplier<Integer> DPAD;
  Rotation2d rotation = new Rotation2d();

  public TurdDrive(TurdSwerve swerve, Supplier<Translation2d> joystickLeft, Supplier<Translation2d> joystickRight, Supplier<Boolean> resetPods, Supplier<Integer> DPAD) {
    this.swerve = swerve;
    this.joystickRight = joystickRight;
    this.joystickLeft = joystickLeft;
    this.resetPods = resetPods;
    this.DPAD = DPAD;
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (resetPods.get()) {
      swerve.resetPods();
    }
    if (DPAD.get() != -1) {
      swerve.targetAngle = -Units.degreesToRadians(DPAD.get());
    }
    boolean deadband = Math.abs(joystickRight.get().getX()) + Math.abs(joystickRight.get().getY()) < Constants.deadband;
    double speedX = deadband ? 0 : -joystickRight.get().getX();
    double speedY = deadband ? 0 : joystickRight.get().getY();
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
