// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/** Add your docs here. */
public class CoralArrayManager {

    public List<CoralObject> selectCoral(List<CoralObject> corals) {
        ArrayList<DoubleSupplier> distances = new ArrayList<>();
        int sizeCorals = corals.size();
        for (int i = 0; i < sizeCorals - 1; i++) {
            double distanceNew = corals.get(i).getDistance();
            distances.add(() -> distanceNew);
        }
        int sizeDistances = distances.size();
        int indexMinDistance = 0;

        for (int i = 1; i < sizeDistances; i++) {
            double distanceMinCurrent = distances.get(indexMinDistance).getAsDouble();
            double distanceMinProspective = distances.get(i).getAsDouble();

            if (distanceMinProspective < distanceMinCurrent) {
                indexMinDistance = i;
            }
        }

        for (int i = 0; i < sizeCorals; i++) {
            if (i != indexMinDistance) {
                CoralObject coralIgnored = corals.get(i);
                coralIgnored.setCoralIgnored(true);
                corals.set(i, coralIgnored);
            } else {
                CoralObject coralTargeted = corals.get(i);
                coralTargeted.setCoralTargeted(true);
                corals.set(i, coralTargeted);
            }
        }

        for (int i = 0; i < sizeCorals; i++) {
            CoralObject coralChecked = corals.get(i);
            if (coralChecked.getIgnored()) {
                corals.remove(i);
                i--;
                sizeCorals = corals.size();
            }
        }

        return corals;
    }

    public List<CoralObject> expiryFilter(List<CoralObject> corals, double hb, double fps) {
        double expiryFrameDiff = fps * 5;
        int sizeCorals = corals.size();

        for(int i = 0; i < sizeCorals; i++) {
            CoralObject coralCompared = corals.get(i);
            double coralHBDiff = Math.abs(coralCompared.getHB() - hb);

            if (coralHBDiff > expiryFrameDiff) {
                CoralObject coralIgnored = corals.get(i);
                coralIgnored.setCoralIgnored(true);
                corals.set(i, coralIgnored);
            }
        }

        for (int i = 0; i < sizeCorals; i++) {
            CoralObject coralChecked = corals.get(i);
            if (coralChecked.getIgnored()) {
                corals.remove(i);
                i--;
                sizeCorals = corals.size();
            }
        }

        return corals;
    }

    public List<CoralObject> displacementFilter(List<CoralObject> corals) {
        double maxDisplacement = 0.0508;
        int sizeCorals = corals.size();

        for (int i = 0; i < sizeCorals - 1; i++) {
            for (int j = i + 1; j < sizeCorals; j++) {
                CoralObject coral1 = corals.get(i);
                CoralObject coral2 = corals.get(j);

                if (!coral1.getIgnored() && !coral2.getIgnored()) {
                    double poseX1 = coral1.getPose().getX();
                    double poseY1 = coral1.getPose().getY();

                    double poseX2 = coral2.getPose().getX();
                    double poseY2 = coral2.getPose().getY();

                    double distance = Math.sqrt(Math.pow((poseX2 - poseX1),2) + Math.pow((poseY2 - poseY1), 2));

                    if (distance <= maxDisplacement) {
                        double hb1 = coral1.getHB();
                        double hb2 = coral2.getHB();

                        if (hb1 < hb2) {
                            CoralObject coralIgnored = coral1;
                            coralIgnored.setCoralIgnored(true);
                            corals.set(i, coralIgnored);
                        } else {
                            CoralObject coralIgnored = coral2;
                            coralIgnored.setCoralIgnored(true);
                            corals.set(j, coralIgnored);
                        }
                    }
                } else if (coral1.getIgnored()) {
                    break;
                }
            }
        }

        for (int i = 0; i < sizeCorals; i++) {
            CoralObject coralChecked = corals.get(i);
            if (coralChecked.getIgnored()) {
                corals.remove(i);
                i--;
                sizeCorals = corals.size();
            }
        }

        return corals;
    }

    public List<CoralObject> distanceAndYawUpdate(List<CoralObject> corals, Pose2d pose) {
        double poseX = pose.getX();
        double poseY = pose.getY();
        Rotation2d poseYaw = pose.getRotation();
        
        int sizeCorals = corals.size();

        for (int i = 0; i < sizeCorals; i++) {
            CoralObject coralToUpdate = corals.get(i);

            Pose2d coralPose = coralToUpdate.getPose();
            
            double coralX = coralPose.getX();
            double coralY = coralPose.getY();

            double distance = Math.sqrt(Math.pow((coralX - poseX), 2) + 
            Math.pow(coralY - poseY, 2));

            Pose2d updatedCoralPose = new Pose2d(coralX, coralY, poseYaw);

            coralToUpdate.setCoralPose(updatedCoralPose);
            coralToUpdate.setCoralDistance(distance);

            corals.set(i, coralToUpdate);
        }

        return corals;
    }

    public boolean getCoralInRange(List<CoralObject> corals, Pose2d pose) {
        boolean coralInRange = false;
        
        double poseX = pose.getX();
        double poseY = pose.getY();

        int sizeCoral = corals.size();

        double minRange = 0.6096; //2 ft in m

        for (int i = 0; i < sizeCoral; i++) {
            CoralObject coralChecked = corals.get(i);

            Pose2d coralPose = coralChecked.getPose();

            double coralX = coralPose.getX();
            double coralY = coralPose.getY();

            double distance = Math.sqrt(Math.pow((coralX - poseX), 2) + 
            Math.pow(coralY - poseY, 2));

            if (distance <= minRange) {
                coralInRange = true;
            }
        }

        return coralInRange;
    }


}
