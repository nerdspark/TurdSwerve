// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.TurdDrive;
import frc.robot.subsystems.TurdPod;

public class RobotContainer {

  public static final XboxController driver = new XboxController(Constants.driverPort);
  public static final TurdPod leftPod = new TurdPod(Constants.leftAzimuthID, Constants.leftDriveID, Constants.leftAbsoluteEncoderID, Constants.leftAzimuthInvert,Constants.rightAzimuthInvert, Constants.leftAbsoluteEncoderOffset);

  public RobotContainer() {
    configureBindings();
    Supplier<Translation2d> driverRightJoystick = () -> new Translation2d(driver.getRightX(), driver.getRightY());
    leftPod.setDefaultCommand(new TurdDrive(leftPod, driverRightJoystick));
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
