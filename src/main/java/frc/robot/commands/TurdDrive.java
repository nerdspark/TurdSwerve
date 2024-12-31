// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

/* import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import frc.robot.constants.RobotMap;
import frc.robot.subsystems.TurdSwerve; */
import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;

import frc.robot.Telemetry;
import frc.robot.constants.Constants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.utils.NerdOdometrySubsystem;
import frc.robot.subsystems.LimeLight;

public class TurdDrive extends Command {
  
  CommandSwerveDrivetrain swerve;
  LimeLight ll;
  CommandXboxController joystick = new CommandXboxController(1);
  // Supplier<Translation2d> joystickRight, joystickLeft;

  // double maxSpeed = Constants.robotMaxSpeed;
  double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps;
  double MaxAngularRate = 1.5 * Math.PI;
  SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  NerdOdometrySubsystem odo = new NerdOdometrySubsystem(swerve);

  Telemetry logger = new Telemetry(MaxSpeed);

  

  public TurdDrive(CommandSwerveDrivetrain swerve, LimeLight ll, CommandXboxController joystick, SwerveRequest.FieldCentric drive, NerdOdometrySubsystem odo, Telemetry logger) {
    this.swerve = swerve;
    this.ll = ll;
    this.joystick = joystick;
    this.drive = drive;
    this.odo = odo;
    this.logger = logger;
/*     this.joystickRight = joystickRight;
    this.joystickLeft = joystickLeft;
    this.DPAD = DPAD;
    this.boost = boost; */
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // swerve.resetGyro();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    swerve.applyRequest(() -> drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        );
    /* if (DPAD.get() != -1) {
      swerve.targetAngle = -Units.degreesToRadians(DPAD.get());
    }

    if (boost.get()) {
      swerve.setAmpLimit(Constants.driveTopAmpLimit);
      maxSpeed = 1;
    } else {
      swerve.setAmpLimit(Constants.driveAmpLimit);
      maxSpeed = Constants.robotMaxSpeed;
    }
    
    boolean deadband = Math.abs(joystickRight.get().getX()) + Math.abs(joystickRight.get().getY()) < 0.05;
    double speedX = deadband ? 0 : -joystickRight.get().getX() * maxSpeed;
    double speedY = deadband ? 0 : joystickRight.get().getY() * maxSpeed;
    // double speedX = deadband ? 0 : 3.0 * Math.abs(joystickRight.get().getX()) * -joystickRight.get().getX();
    // double speedY = deadband ? 0 : 3.0 * Math.abs(joystickRight.get().getY()) * joystickRight.get().getY();
    double speedOmega = Math.abs(joystickLeft.get().getX()) > 0.07 ? -joystickLeft.get().getX() * Math.abs(joystickLeft.get().getX())*0.3 : 0;
    ChassisSpeeds speeds = new ChassisSpeeds(speedX, speedY, speedOmega);
    SmartDashboard.putNumber("Execute-SpeedX", speedX);
    SmartDashboard.putNumber("Execute-SpeedY", speedY);
    SmartDashboard.putNumber("tX", ll.getTx());
    SmartDashboard.putNumber("tY", ll.getTy());
    SmartDashboard.putNumber("tA", ll.getTa());
    swerve.setRobotSpeeds(speeds); */
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
