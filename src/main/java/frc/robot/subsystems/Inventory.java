// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Inventory extends SubsystemBase {
  /** Creates a new Inventory. */
  private boolean A = false;
  private boolean B = false;
  private boolean X = false;
  private boolean Y = false;
  public Inventory() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public boolean getAstatus() {
    return A;
  }
  public boolean getBstatus() {
    return B;
  }
  public boolean getXstatus() {
    return X;
  }
  public boolean getYstatus() {
    return Y;
  }

  public void setAstatus(boolean status) {
    A = status;
  }

  public void setBstatus(boolean status) {
    B = status;
  }

  public void setXstatus(boolean status) {
    X = status;
  }

  public void setYstatus(boolean status) {
    Y = status;
  }
}
