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
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.MoveSequence;
import frc.robot.commands.ResetZeroes;
import frc.robot.commands.RevertZeroes;
import frc.robot.commands.TurdDrive;
import frc.robot.commands.TurdDriveAuto;
import frc.robot.commands.TurdFollowAprilTag;
import frc.robot.commands.TurdPose;
import frc.robot.constants.Constants;
import frc.robot.subsystems.TurdSwerve;
import frc.robot.subsystems.Inventory;
import frc.robot.subsystems.LimeLight;

public class RobotContainer {

  public static final XboxController driverRaw = new XboxController(Constants.driverPort);
  public static final CommandXboxController driverCommand = new CommandXboxController(Constants.driverPort);
  // public static final TurdPod leftPod = new TurdPod(Constants.leftAzimuthID, Constants.leftDriveID, Constants.leftAbsoluteEncoderID, Constants.leftAzimuthInvert,Constants.rightAzimuthInvert, Constants.leftAbsoluteEncoderOffset);
  public static final TurdSwerve swerve = new TurdSwerve();
  public static final LimeLight ll = new LimeLight();
  public static final Inventory inventory = new Inventory();
  JoystickButton A = new JoystickButton(driverRaw, 1);
  JoystickButton B = new JoystickButton(driverRaw, 2);
  

  public RobotContainer() {
    final var Odometry = Shuffleboard.getTab("Odometry");
    configureBindings();
    Supplier<Translation2d> driverRightJoystick = () -> new Translation2d(driverRaw.getRightX(), driverRaw.getRightY());
    Supplier<Translation2d> driverLeftJoystick = () -> new Translation2d(driverRaw.getLeftX(), driverRaw.getLeftY());
    Supplier<Integer> DPAD = () -> driverRaw.getPOV();
    
    
    swerve.setDefaultCommand(new TurdDrive(swerve, ll, driverLeftJoystick, driverRightJoystick, DPAD, driverRaw::getLeftBumper, inventory));
    // A.toggleOnTrue(new TurdFollowAprilTag(swerve, ll));
    // B.toggleOnTrue(new TurdPose(swerve, ll));

    driverCommand.a().whileTrue(new InstantCommand(() -> inventory.setAstatus(true)));
    driverCommand.b().whileTrue(new InstantCommand(() -> inventory.setBstatus(true)));
    driverCommand.x().whileTrue(new InstantCommand(() -> inventory.setXstatus(true)));
    driverCommand.y().whileTrue(new InstantCommand(() -> inventory.setYstatus(true)));

    driverCommand.a().whileFalse(new InstantCommand(() -> inventory.setAstatus(false)));
    driverCommand.b().whileFalse(new InstantCommand(() -> inventory.setBstatus(false)));
    driverCommand.x().whileFalse(new InstantCommand(() -> inventory.setXstatus(false)));
    driverCommand.y().whileFalse(new InstantCommand(() -> inventory.setYstatus(false)));
    //A.onTrue(new MoveSequence(swerve));

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
