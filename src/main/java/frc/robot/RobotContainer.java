// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.ResetZeroes;
import frc.robot.commands.RevertZeroes;
import frc.robot.commands.TurdDrive;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.utils.NerdOdometrySubsystem;

import frc.robot.subsystems.LimeLight;

public class RobotContainer {
  private double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
  private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandSwerveDrivetrain swerve = TunerConstants.DriveTrain; // My drivetrain
  public static final LimeLight ll = new LimeLight();
  private final CommandXboxController joystick = new CommandXboxController(1); // My joystick
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  private final NerdOdometrySubsystem odo = new NerdOdometrySubsystem(swerve);

  private final Telemetry logger = new Telemetry(MaxSpeed);
  

  /* public static final XboxController driverRaw = new XboxController(Constants.driverPort);
  public static final CommandXboxController driverCommand = new CommandXboxController(Constants.driverPort);
  // public static final TurdPod leftPod = new TurdPod(Constants.leftAzimuthID, Constants.leftDriveID, Constants.leftAbsoluteEncoderID, Constants.leftAzimuthInvert,Constants.rightAzimuthInvert, Constants.leftAbsoluteEncoderOffset);
  public static final TurdSwerve swerve = new TurdSwerve();
  
  JoystickButton A = new JoystickButton(driverRaw, 1);
  JoystickButton B = new JoystickButton(driverRaw, 2);
  JoystickButton X = new JoystickButton(driverRaw, 3); */
  

  public RobotContainer() {
    /* final var Odometry = Shuffleboard.getTab("Odometry");
    configureBindings();
    Supplier<Translation2d> driverRightJoystick = () -> new Translation2d(driverRaw.getRightX(), driverRaw.getRightY());
    Supplier<Translation2d> driverLeftJoystick = () -> new Translation2d(driverRaw.getLeftX(), driverRaw.getLeftY());
    Supplier<Integer> DPAD = () -> driverRaw.getPOV(); */
    
    
    swerve.setDefaultCommand(new TurdDrive(swerve, ll, joystick, drive, odo, logger));
 /*    A.toggleOnTrue(new TurdFollowAprilTag(swerve, ll));
    B.toggleOnTrue(new TurdPose(swerve, ll));
    X.toggleOnTrue(new TurdToBucket(swerve, ll)); */
    //A.onTrue(new MoveSequence(swerve));

      //swerve.addDashboardWidgets(Odometry);
    
  

    
  }

  private void configureBindings() {
    /* driverCommand.rightBumper().and(driverRaw::getYButton).onTrue(new ResetZeroes(swerve));
    driverCommand.rightBumper().and(driverRaw::getXButton).whileTrue(new RevertZeroes(swerve));
    driverCommand.start().whileTrue(new InstantCommand(swerve::resetPods, swerve)); */
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
