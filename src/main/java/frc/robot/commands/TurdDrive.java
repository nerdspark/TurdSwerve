// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TurdPod;

public class TurdDrive extends CommandBase {
  
  TurdPod pod;
  Supplier<Translation2d> joystickRight;

  public TurdDrive(TurdPod pod, Supplier<Translation2d> joystickRight) {
    this.pod = pod;
    this.joystickRight = joystickRight;
    addRequirements(pod);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SwerveModuleState state = new SwerveModuleState(joystickRight.get().getX(), new Rotation2d(joystickRight.get().getY()));
    pod.setPodState(state);
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
