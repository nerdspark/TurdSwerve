// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

/** Add your docs here. */
import java.lang.reflect.Array;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.AutoDriveConstants;
import frc.robot.subsystems.Inventory;

/** Add your docs here. */
public class PIDToPosition {
    private PIDController robotPID = new PIDController(1.0, 0, 0);

    // public Translation2d CalculatePID(Pose2d position) {
    //     // Translation2d[] translations = new Translation2d[1];
    //     //Translation2d translations = new Translation2d(-robotPID.calculate(position.getX(), AutoDriveConstants.positionA.getX()), -robotPID.calculate(position.getY(), AutoDriveConstants.positionA.getY()));
    //     // translations[1] = new Translation2d(-robotPID.calculate(position.getX(), AutoDriveConstants.positionB.getX()), -robotPID.calculate(position.getY(), AutoDriveConstants.positionB.getY()));
    //     // translations[2] = new Translation2d(-robotPID.calculate(position.getX(), AutoDriveConstants.positionX.getX()), -robotPID.calculate(position.getY(), AutoDriveConstants.positionX.getY()));
    //     // translations[3] = new Translation2d(-robotPID.calculate(position.getX(), AutoDriveConstants.positionY.getX()), -robotPID.calculate(position.getY(), AutoDriveConstants.positionY.getY()));
    //     //return translations;
    // }


    public Translation2d ChooseVector(Pose2d position, boolean inventory) {
        if (inventory == true){
            Translation2d[] positions = new Translation2d[3];
            positions[0] = AutoDriveConstants.position1;
            positions[1] = AutoDriveConstants.position2;
            positions[2] = AutoDriveConstants.position3;
            double[] differences = new double[3];
            differences[0] = position.getTranslation().getDistance(AutoDriveConstants.position1);
            differences[1] = position.getTranslation().getDistance(AutoDriveConstants.position2);
            differences[2] = position.getTranslation().getDistance(AutoDriveConstants.position3);
            SmartDashboard.putNumber("1, 0", differences[0]);
            SmartDashboard.putNumber("-1, 0", differences[1]);
            SmartDashboard.putNumber("0, 1", differences[2]);
            double bestDistance = differences[1];
            int selecter = 0;
            for (int i = 0; i < differences.length; i++){
                if(differences[i] < bestDistance){
                    bestDistance = differences[i];
                    selecter = i;
                }
            }
            if(bestDistance < 2){
                return positions[selecter];
            }else{
                return position.getTranslation();
            }
        }
        return position.getTranslation();
        

    }
    // public boolean ActivationZone(Pose2d position, Translation2d drive){
    //     boolean activateA = position.getTranslation().getDistance(AutoDriveConstants.position1) < 0.5;
    //     boolean activateB = position.getTranslation().getDistance(AutoDriveConstants.position2) < 0.5;
    //     boolean activateC = position.getTranslation().getDistance(AutoDriveConstants.position3) < 0.5;
    //     // boolean activateB = position.getTranslation().getDistance(AutoDriveConstants.positionB) < 3;
    //     // boolean activateX = position.getTranslation().getDistance(AutoDriveConstants.positionX) < 3;
    //     // boolean activateY = position.getTranslation().getDistance(AutoDriveConstants.positionY) < 3;
    //     return activ;
    // }



}
