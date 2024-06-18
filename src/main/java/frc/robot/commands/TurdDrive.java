// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants.RobotConfig;
import frc.robot.subsystems.TurdSwerve;

public class TurdDrive extends Command {
    
    TurdSwerve swerve;
    Supplier<Translation2d> joystickRight, joystickLeft;
    Supplier<Integer> DPAD;
    Supplier<Boolean> boost;
    Rotation2d rotation = new Rotation2d();
    double maxSpeed = RobotConfig.robotMaxSpeed;

    
    private final SlewRateLimiter xLimiter = new SlewRateLimiter(0.75);
    private final SlewRateLimiter yLimiter = new SlewRateLimiter(0.75);

    public TurdDrive(TurdSwerve swerve, Supplier<Translation2d> joystickLeft, Supplier<Translation2d> joystickRight, Supplier<Integer> DPAD, Supplier<Boolean> boost) {
        this.swerve = swerve;
        this.joystickRight = joystickRight;
        this.joystickLeft = joystickLeft;
        this.DPAD = DPAD;
        this.boost = boost;
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

        if (boost.get()) {
            swerve.setAmpLimit(RobotConfig.boostDriveLimit);
            maxSpeed = 1;
        } else {
            swerve.setAmpLimit(RobotConfig.driveAmpLimit);
            maxSpeed = RobotConfig.robotMaxSpeed;
        }

        //notice that X and Y are flipped - this is due to differences between WPILib field-based orientation (+Y is field "up") and conventional robot-based orientation (+Y is robot forward)
        double speedX = MathUtil.applyDeadband(-joystickRight.get().getY(), 0.05) * maxSpeed; // Y is negated due to the way controllers work
        double speedY = MathUtil.applyDeadband(-joystickRight.get().getX(), 0.05) * maxSpeed; // X is negated due to the way field oriented works (assuming a blue origin, "right" would be negative Y)

        double speedOmega = MathUtil.applyDeadband(joystickLeft.get().getX(), 0.07);
        ChassisSpeeds speeds = new ChassisSpeeds(xLimiter.calculate(speedX), yLimiter.calculate(speedY), speedOmega);
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
