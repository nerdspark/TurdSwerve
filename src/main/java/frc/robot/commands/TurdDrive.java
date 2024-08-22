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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.constants.AutoDriveConstants;
import frc.robot.constants.Constants;
import frc.robot.constants.RobotMap;
import frc.robot.subsystems.TurdSwerve;
import frc.robot.util.PIDToPosition;
import frc.robot.subsystems.Inventory;
import frc.robot.subsystems.LimeLight;

public class TurdDrive extends Command {
  
  TurdSwerve swerve;
  LimeLight ll;
  Supplier<Translation2d> joystickRight, joystickLeft;
  Supplier<Integer> DPAD;
  Supplier<Boolean> boost;
  Rotation2d rotation = new Rotation2d();
  double maxSpeed = Constants.robotMaxSpeed;
  PIDToPosition PID = new PIDToPosition();
  Inventory inventory;

  public TurdDrive(TurdSwerve swerve, LimeLight ll, Supplier<Translation2d> joystickLeft, Supplier<Translation2d> joystickRight, Supplier<Integer> DPAD, Supplier<Boolean> boost, Inventory inventory) {
    this.swerve = swerve;
    this.ll = ll;
    this.joystickRight = joystickRight;
    this.joystickLeft = joystickLeft;
    this.DPAD = DPAD;
    this.boost = boost;
    this.inventory = inventory;
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


    boolean[] inventoryStatuses = {inventory.getAstatus(), inventory.getBstatus(), inventory.getXstatus(), inventory.getYstatus()};
    SmartDashboard.putBooleanArray("inventory", inventoryStatuses);
    Translation2d[] PIDVectors = PID.CalculatePID(swerve.odometer.getPoseMeters());
    PIDVectors = PID.FilterVectors(PIDVectors, inventoryStatuses);
    Translation2d BestVector = PID.ChooseVector(swerve.odometer.getPoseMeters(), new Translation2d(joystickRight.get().getY(), -joystickRight.get().getX()), PIDVectors);
    boolean deadzoneA = swerve.odometer.getPoseMeters().getTranslation().getDistance(AutoDriveConstants.positionA) < AutoDriveConstants.zone;
    boolean deadzoneB = swerve.odometer.getPoseMeters().getTranslation().getDistance(AutoDriveConstants.positionB) < AutoDriveConstants.zone;
    boolean deadzoneX = swerve.odometer.getPoseMeters().getTranslation().getDistance(AutoDriveConstants.positionX) < AutoDriveConstants.zone;
    boolean deadzoneY = swerve.odometer.getPoseMeters().getTranslation().getDistance(AutoDriveConstants.positionY) < AutoDriveConstants.zone;
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

    // BestVector = BestVector.getNorm() > 0.2 ? new Translation2d(0.2, BestVector.getAngle()) : (BestVector.getNorm() < -0.2 ? new Translation2d(-0.2, BestVector.getAngle()) : BestVector);




    if (DPAD.get() != -1) {
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


    if (BestVector.getNorm() > 0.01) {
    swerve.setRobotSpeeds(new ChassisSpeeds(BestVector.getY(), -BestVector.getX(), 0));
    } else {
    swerve.setRobotSpeeds(speeds);
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
