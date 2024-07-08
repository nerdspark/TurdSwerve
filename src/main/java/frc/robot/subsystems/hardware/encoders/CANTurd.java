// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.hardware.encoders;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

public class CANTurd implements TurdCoder {
    private CANcoder encoder;

    // any edits to the CANcoder configuration should be done through this variable
    private CANcoderConfiguration config = new CANcoderConfiguration();

    //TODO: this is a "testing only" variable. please delete and de-implement it when possible.
    private final double initialOffset;


    /**
     * creates a new CANTurdCoder
     * @param inverted true for CW+, false for CCW+
     * @param offset the offset of the sensor in rotations
     * @param id the CAN id of the sensor
     */
    public CANTurd(int id, boolean inverted, double offset) {
        encoder = new CANcoder(id);

        config.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
        config.MagnetSensor.SensorDirection = inverted ? SensorDirectionValue.Clockwise_Positive : SensorDirectionValue.CounterClockwise_Positive;
        config.MagnetSensor.MagnetOffset = -offset;

        //not applying magnet config directly in order to overwrite other settings
        applyConfig();

        initialOffset = offset;
    }

    public CANTurd(int id, double offset) {
        this(id, false, offset);
    }

    /**
     * applies a configuration to the encoder
     * @param config configuration to apply
     * 
     * @implNote please use {@link #getConfig()} to get the current configuration, modify it, and then apply it
     */
    private void applyConfig() {
        encoder.getConfigurator().apply(config);
    }

    /**
     * @return the current configuration of the encoder
     */
    public CANcoderConfiguration getConfig() {
        return config;
    }

    @Override
    public double getAbsoluteAngle() {
        return encoder.getAbsolutePosition().getValue();
    }

    @Override
    public void setPosition(double value) {
        encoder.setPosition(value);
    }

    @Override
    public void revertZero() {
        if(config.MagnetSensor.MagnetOffset != initialOffset) {
            config.MagnetSensor.MagnetOffset = initialOffset; 
            applyConfig();
        }
    }
}