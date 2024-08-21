// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import java.lang.reflect.Array;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.constants.AutoDriveConstants;
import frc.robot.subsystems.Inventory;

/** Add your docs here. */
public class PIDToPosition {
    private PIDController robotPID = new PIDController(3.0, 0, 0);

    public Translation2d[] CalculatePID(Pose2d position) {
        Translation2d[] translations = new Translation2d[4];
        translations[0] = new Translation2d(-robotPID.calculate(position.getX(), AutoDriveConstants.positionA.getX()), -robotPID.calculate(position.getY(), AutoDriveConstants.positionA.getY()));
        translations[1] = new Translation2d(-robotPID.calculate(position.getX(), AutoDriveConstants.positionB.getX()), -robotPID.calculate(position.getY(), AutoDriveConstants.positionB.getY()));
        translations[2] = new Translation2d(-robotPID.calculate(position.getX(), AutoDriveConstants.positionX.getX()), -robotPID.calculate(position.getY(), AutoDriveConstants.positionX.getY()));
        translations[3] = new Translation2d(-robotPID.calculate(position.getX(), AutoDriveConstants.positionY.getX()), -robotPID.calculate(position.getY(), AutoDriveConstants.positionY.getY()));
        return translations;
    }

    public Translation2d[] FilterVectors(Translation2d[] translations, boolean[] inventory) {
        for (int i = 0; i < translations.length; i++) {
            if (!inventory[i]) {
                translations[i] = new Translation2d();
            }
        }
        return translations;
    }

    public Translation2d ChooseVector(Pose2d position, Translation2d drive, Translation2d[] translations) {
        double[] differences = new double[4];
        for (int i = 0; i < 4; i++) {
            differences[i] = Math.abs(translations[i].getAngle().minus(drive.getAngle()).getRadians());
        }

        double minRotation = Math.PI;
        int bestValue = 10000;
        for (int i = 0; i < 4; i++) {
            if (differences[i] < minRotation) {
                minRotation = differences[i];
                if (!translations[i].equals(new Translation2d())) {
                    bestValue = i;
                }
            }
        }
        return bestValue == 10000 ? new Translation2d() : translations[bestValue];
    }



}
