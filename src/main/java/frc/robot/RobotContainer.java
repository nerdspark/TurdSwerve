// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.ResetZeroes;
import frc.robot.commands.RevertZeroes;
import frc.robot.commands.TurdDrive;
import frc.robot.constants.Constants;
import frc.robot.subsystems.TurdSwerve;

public class RobotContainer {

  public static final XboxController driverRaw = new XboxController(Constants.driverPort);
  public static final CommandXboxController driverCommand = new CommandXboxController(Constants.driverPort);
  public static final TurdSwerve swerve = new TurdSwerve();
  

  public RobotContainer() {
    final var Odometry = Shuffleboard.getTab("Odometry");
    configureBindings();
    Supplier<Translation2d> driverRightJoystick = () -> new Translation2d(driverRaw.getRightX(), driverRaw.getRightY());
    Supplier<Translation2d> driverLeftJoystick = () -> new Translation2d(driverRaw.getLeftX(), driverRaw.getLeftY());
    Supplier<Integer> DPAD = () -> driverRaw.getPOV();
    swerve.setDefaultCommand(new TurdDrive(swerve, driverLeftJoystick, driverRightJoystick, DPAD, driverRaw::getLeftBumper));
    swerve.addDashboardWidgets(Odometry);
  }

  private void configureBindings() {
    driverCommand.rightBumper().and(driverRaw::getYButton).onTrue(new ResetZeroes(swerve));
    driverCommand.rightBumper().and(driverRaw::getXButton).whileTrue(new RevertZeroes(swerve));
    driverCommand.start().whileTrue(new InstantCommand(swerve::resetPods, swerve));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
