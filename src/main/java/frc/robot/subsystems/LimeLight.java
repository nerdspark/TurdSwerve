// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class LimeLight extends SubsystemBase {
  private final NetworkTable llTable;

  public LimeLight() {
    llTable = NetworkTableInstance.getDefault().getTable("limelight");
  }

  public double getTx() {
    return llTable.getEntry("tx").getDouble(0.0);
  }

  public double getTy() {
    return llTable.getEntry("ty").getDouble(0.0);
  }

  public double getTa() {
    return llTable.getEntry("ta").getDouble(0.0);
  }

  public boolean hasTarget() {
    return llTable.getEntry("tv").getDouble(0.0) == 1.0;
  }
  
}
