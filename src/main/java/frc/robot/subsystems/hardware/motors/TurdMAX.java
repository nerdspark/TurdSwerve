// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.hardware.motors;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

/** Add your docs here. */
public class TurdMAX implements TurdMotor {
    CANSparkMax motor;
    RelativeEncoder encoder;
    SparkPIDController PID;

    public TurdMAX(int id, double positionConversionFactor, int currentLimit, boolean invert, boolean isBrake, double rampRate) {
        motor = new CANSparkMax(id, MotorType.kBrushless);
        
        encoder = motor.getEncoder();
        encoder.setPositionConversionFactor(positionConversionFactor);
        
        motor.setInverted(invert);
        motor.setSmartCurrentLimit(currentLimit);
        motor.setIdleMode(isBrake ? IdleMode.kBrake : IdleMode.kCoast);

        motor.setOpenLoopRampRate(rampRate);

        PID = motor.getPIDController();
    }

    public TurdMAX(int id, double positionConversionFactor, int currentLimit, boolean invert, boolean isBrake, double rampRate, double kP, double kI, double kD, double Iz, double maxOutput) {
        this(id, positionConversionFactor, currentLimit, invert, isBrake, rampRate);
        setPID(Iz, kP, kI, kD, maxOutput);
    }

    @Override
    public void setPower(double power) {
        motor.set(power);
    }

    @Override
    public double getPosition() {
        return encoder.getPosition();
    }

    @Override
    public void setPosition(double value) {
        encoder.setPosition(value);
    }

    @Override
    public void setTargetPosition(double target) {
        PID.setReference(target, ControlType.kPosition);
    }

    @Override
    public void setPID(double IZone, double P, double I, double D, double outputRange) {
        if (P != PID.getP()) {PID.setP(P);}
        if (I != PID.getI()) {PID.setI(I);}
        if (D != PID.getD()) {PID.setD(D);}
        if (IZone != PID.getIZone()) {PID.setIZone(IZone);}
        if (outputRange != PID.getOutputMax()) {PID.setOutputRange(-outputRange, outputRange);}
        PID.setPositionPIDWrappingMaxInput(Math.PI);
        PID.setPositionPIDWrappingMinInput(-Math.PI);
        PID.setPositionPIDWrappingEnabled(true);
    }

    @Override
    public void setAmpLimit(int limit) {
        motor.setSmartCurrentLimit(limit);
    }

    @Override
    public double getAppliedOutput() {
        return motor.getAppliedOutput();
    }

    @Override
    public double getState() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getState'");
    }
}
