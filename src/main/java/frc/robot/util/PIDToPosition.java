// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.constants.AutoDriveConstants;
import frc.robot.subsystems.Inventory;

/** Add your docs here. */
public class PIDToPosition {
    private PIDController robotPID = new PIDController(0, 0, 0);

    private Translation2d[] CalculatePID(Pose2d position) {
        Translation2d[] translation = new Translation2d[3];
        translation[0] = new Translation2d(robotPID.calculate(position.getX(), AutoDriveConstants.positionA.getX()), robotPID.calculate(position.getY(), AutoDriveConstants.positionA.getY()));
        translation[1] = new Translation2d(robotPID.calculate(position.getX(), AutoDriveConstants.positionB.getX()), robotPID.calculate(position.getY(), AutoDriveConstants.positionB.getY()));
        translation[2] = new Translation2d(robotPID.calculate(position.getX(), AutoDriveConstants.positionX.getX()), robotPID.calculate(position.getY(), AutoDriveConstants.positionX.getY()));
        translation[3] = new Translation2d(robotPID.calculate(position.getX(), AutoDriveConstants.positionY.getX()), robotPID.calculate(position.getY(), AutoDriveConstants.positionY.getY()));
        return translation;
    }

    public Translation2d ChoosePID(Pose2d position, Translation2d drive) {
        Translation2d[] translations = CalculatePID(position);
        double[] differences = new double[3];
        for (int i = 0; i < 4; i++) {
            differences[i] = Math.abs(translations[i].getAngle().minus(drive.getAngle()).getRadians());
        }

        double minRotation = Math.PI;
        int bestValue = 0;
        for (int i = 0; i < 4; i++) {
            if (differences[i] < minRotation) {
                minRotation = differences[i];
                bestValue = i;
            }
        }
        return translations[bestValue];
    }



}
