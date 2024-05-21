// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.hardware.encoders;

import edu.wpi.first.wpilibj.AnalogEncoder;

public class ThrifTurd implements TurdCoder{
    private AnalogEncoder encoder;
    private double offset;

    //TODO: this is a "testing only" variable. please delete and de-implement it when possible.
    private final double initialOffset;

    /**
     * creates a new ThrifTurdCoder
     * @param inverted whether the encoder is inverted
     * @param offset the offset of the encoder in radians
     * @param conversionFactor the conversion factor of the encoder
     * @param id the id of the encoder
     */
    public ThrifTurd(int id, double offset, double conversionFactor){
        encoder = new AnalogEncoder(id);
        this.offset = offset;
        initialOffset = offset;

        encoder.setDistancePerRotation(conversionFactor);        
    }

    @Override
    public double getAbsoluteAngle() {
        return (encoder.getAbsolutePosition() * (Math.PI * 2)) - offset;
    }

    @Override
    public void setPosition(double value) {
        //TODO: i believe it should be plus value, but it might be minus value
        offset = (encoder.getAbsolutePosition() * (Math.PI * 2)) + value;
    }

    @Override
    public void revertZero() {
        offset = initialOffset;
    }
}
