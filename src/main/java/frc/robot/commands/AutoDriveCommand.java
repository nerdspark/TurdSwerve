// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.*;
import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.AutoDriveConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Inventory;
import frc.robot.util.PIDToPosition;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoDriveCommand extends Command {
  PIDToPosition PID = new PIDToPosition();
  CommandSwerveDrivetrain swerve;
  Inventory inventory;
  Supplier<Translation2d> joystickLeft;
  Supplier<Translation2d> joystickRight;
  double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);
  public final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric();
  
  /** Creates a new AutoDriveCommand. */
  public AutoDriveCommand(CommandSwerveDrivetrain swerve, Supplier<Translation2d> joystickLeft, Supplier<Translation2d> joystickRight, Inventory inventory) {
    this.swerve = swerve;
    this.inventory = inventory;
    this.joystickLeft = joystickLeft;
    this.joystickRight = joystickRight;
    //this.drive = drive;
    addRequirements(swerve);
    SmartDashboard.putBoolean("boolean", true);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    boolean[] inventoryStatuses = {inventory.getAstatus(), inventory.getBstatus(), inventory.getXstatus(), inventory.getYstatus()};
    SmartDashboard.putBooleanArray("inventory", inventoryStatuses);
    Translation2d[] PIDVectors = PID.CalculatePID(swerve.getState().Pose);
    PIDVectors = PID.FilterVectors(PIDVectors, inventoryStatuses);
    Translation2d BestVector = PID.ChooseVector(swerve.getState().Pose, new Translation2d(joystickLeft.get().getY(), -joystickLeft.get().getX()), PIDVectors);
    boolean deadzoneA = swerve.getState().Pose.getTranslation().getDistance(AutoDriveConstants.positionA) < AutoDriveConstants.zone;
    boolean deadzoneB = swerve.getState().Pose.getTranslation().getDistance(AutoDriveConstants.positionB) < AutoDriveConstants.zone;
    boolean deadzoneX = swerve.getState().Pose.getTranslation().getDistance(AutoDriveConstants.positionX) < AutoDriveConstants.zone;
    boolean deadzoneY = swerve.getState().Pose.getTranslation().getDistance(AutoDriveConstants.positionY) < AutoDriveConstants.zone;
    if (deadzoneA == true){
      inventory.setAstatus(false);
    }
    if (deadzoneB == true){
      inventory.setBstatus(false);
    }
    if (deadzoneX == true){
      inventory.setXstatus(false);
    }
    if (deadzoneY == true){
      inventory.setYstatus(false);
    }
    if (BestVector.getNorm() > 0.01) {
      swerve.applyRequest(() -> drive.withVelocityX(-BestVector.getY() * MaxSpeed*0.1).withVelocityY(-BestVector.getX() * MaxSpeed*0.1).withRotationalRate(0.0));
    } else {
      swerve.applyRequest(() ->
      drive.withVelocityX(-joystickLeft.get().getY() * MaxSpeed*0.1)
                    .withVelocityY(-joystickLeft.get().getX() * MaxSpeed*0.1)
                    .withRotationalRate(-joystickRight.get().getX() * MaxAngularRate));
    }
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
