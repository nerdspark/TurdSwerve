// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.hardware.encoders;

public interface TurdCoder {
    /**
     * @return the absolute angle of the encoder in radians
     */
    public double getAbsoluteAngle();
    
    /**
     * sets the encoder to the specified value (automatically convertts from radians to native units)
     * @param value the value to set the encoder to in radians
     */
    public void setPosition(double value);


    //TODO: this seems to be a "testing only" method. please delete and de-implement it when possible.
    public void revertZero();
}
