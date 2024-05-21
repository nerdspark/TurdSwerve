// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.hardware.motors;

/** Add your docs here. */
public interface TurdMotor {
    /**
     * sets the encoder to the specified value (position conversion factor is handled by the motor, likely radians here)
     * @param value the value to set the encoder
     */
    public void setPosition(double value);

    /**
     * sets the PID target to the specified value (position conversion factor is handled by the motor, likely radians here)
     * @param value the value to set the motor
     * @see #setPID(double, double, double, double, double)
     */
    public void setTargetPosition(double value);

    /**
     * applies the specified power to the motor
     * @param power the power to set to the motor from -1 to 1
     */
    public void setPower(double power);

    /**
     * @return the output reported by the motor's encoder (likely meters for drive and radians for turn)
     */
    public double getPosition();

    /**
     * sets the PID values of the motor (likely used for azimuth)
     * @param wildcard a wildcard value that is used for the motor's specific PID controller. For Talons, it is kS, and for Sparks, it is the IZone.
     * @implNote please note that P is not the first parameter.
     */
    public void setPID(double wildcard, double P, double I, double D, double outputRange);

    /**
     * sets the current limit of the motor
     * @param limit the current limit to set to the motor
     */
    public void setAmpLimit(int limit); // honestly this feels kind of silly but you guys want a boost button so here you go

    /**
     * @return the current output of the motor
     */
    public double getAppliedOutput();

    //TODO: I really believe we should implement velocity control on the drive motor. autons.

}
