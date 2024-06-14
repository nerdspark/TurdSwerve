// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.hardware.motors;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation2d;

/** Add your docs here. */
public class TurdonFX implements TurdMotor {
    TalonFX motor;
    TalonFXConfiguration config = new TalonFXConfiguration();
    TalonFXConfiguration lastAppliedConfig = config;

    public double target = 0;
    boolean apply = true;

    //making position voltage default because it's the simplest. if you want to use a different control type, you can change it
    private final PositionDutyCycle anglePID = new PositionDutyCycle(0).withSlot(0);


    /**
     * Creates a new TurdonFX (please use this for drive motors only)
     * @param id CAN ID for the motor
     * @param inverted true for CW+, false for CCW+
     * @param isBrake true for brake, false for coast
     * @param statorLimit the stator current limit in amps
     * @param rampRate time it takes for the motor to reach full power from zero power in seconds
     * @param ENCODER_TO_MECHANISM_RATIO ratio between the feedback encoder (integrated for drive motors) and the mechanism. this varies based on your gear ratio
     * @param ROTOR_TO_ENCODER_RATIO ratio between the rotor and the feedback encoder. this is usually 1 for drive motors
     */
    public TurdonFX(int id, boolean inverted, boolean isBrake, double statorLimit, double rampRate, double ENCODER_TO_MECHANISM_RATIO, double ROTOR_TO_ENCODER_RATIO) {
        //I figured nobody had the guts to put a CANivore on a turdswerve, so i'm leaving out the CAN bus parameter
        motor = new TalonFX(id);

        //set neutral mode and inverts
        config.MotorOutput.Inverted = inverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = isBrake ? NeutralModeValue.Brake : NeutralModeValue.Coast;

        //set current limits; supply current limits are hardcoded because they are almost always the same
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 40d;
        config.CurrentLimits.SupplyCurrentThreshold = 40d;
        config.CurrentLimits.SupplyTimeThreshold = 100d;

        config.CurrentLimits.StatorCurrentLimitEnable = statorLimit > 0;
        config.CurrentLimits.StatorCurrentLimit = statorLimit;


        // this is kind of bad code, but it's the easiest way to set a ramp rate regardless of control type
        this.config.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = rampRate;
        this.config.OpenLoopRamps.TorqueOpenLoopRampPeriod = rampRate;
        this.config.OpenLoopRamps.VoltageOpenLoopRampPeriod = rampRate; 
        this.config.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = rampRate;
        this.config.ClosedLoopRamps.TorqueClosedLoopRampPeriod = rampRate;
        this.config.ClosedLoopRamps.VoltageClosedLoopRampPeriod = rampRate;

        //set feedback ratios
        config.Feedback.SensorToMechanismRatio = ENCODER_TO_MECHANISM_RATIO;
        config.Feedback.RotorToSensorRatio = ROTOR_TO_ENCODER_RATIO;
        //the remote sensor defaults to internal encoder

        applyConfig();
    }

    /**
     * Creates a new TurdonFX (please use this for azimuth with fused CANcoders only)
     * @param id CAN ID for the motor
     * @param inverted true for CW+, false for CCW+
     * @param isBrake true for brake, false for coast
     * @param statorLimit the stator current limit in amps
     * @param rampRate time it takes for the motor to reach full power from zero power in seconds
     * @param outputRange the min/max output of the motor
     * @param angleEncoderID the id of the CANcoder used fused with the motor
     * @param ENCODER_TO_MECHANISM_RATIO Ratio between the feedback encoder (CANcoder) and mechanism. This is usually 1 for azimuth motors.
     * @param ROTOR_TO_ENCODER_RATIO Ratio between the rotor and the feedback encoder. This is depends on the gearbox.
     * 
     * @implNote this constructor is for azimuth motors only and uses fused CANcoders. If you are not using CANcoders or do not have phoenix pro, please use another constructor
     */
    public TurdonFX(int id, boolean inverted, boolean isBrake, double statorLimit, double rampRate, double ENCODER_TO_MECHANISM_RATIO, double ROTOR_TO_ENCODER_RATIO, double P, double I, double D, double kF, double outputRange, int angleEncoderID) {
        this(id, inverted, isBrake, statorLimit, rampRate, ENCODER_TO_MECHANISM_RATIO, ROTOR_TO_ENCODER_RATIO);
        setPID(kF, P, I, D, outputRange);
        config.ClosedLoopGeneral.ContinuousWrap = true;
        config.Feedback.FeedbackRemoteSensorID = angleEncoderID;
        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;

        applyConfig();
    }

    //TODO: make non-fused-CANcoder azimuth constructor

    private void applyConfig(TalonFXConfiguration config) {
        //only change the config if it's different from the last applied config
        motor.getConfigurator().apply(config);
        lastAppliedConfig = config;

        System.out.println("CONFIG APPLIED ______________________________________");
    }

    private void applyConfig() {
        //only change the config if it's different from the last applied config
        // if(!config.equals(lastAppliedConfig)) {
            motor.getConfigurator().apply(config);
            // lastAppliedConfig = config;

            // System.out.println("CONFIG APPLIED ______________________________________");
        // }
    }

    @Override
    public void setPower(double power) {
        motor.set(power);
    }

    @Override
    public double getPosition() {
        return motor.getPosition().getValueAsDouble() * Math.PI * 2;
    }
    @Override
    public void setPosition(double value) {
        motor.setPosition(value);
    }

    @Override
    public void setTargetPosition(double target) {
        motor.setControl(anglePID.withPosition(target));
    }

    @Override
    public void setPID(double kS, double P, double I, double D, double outputRange) {
        if(config.Slot0.kP != P) {config.Slot0.kP = P; apply = true;}
        if(config.Slot0.kI != I) {config.Slot0.kI = I; apply = true;}
        if(config.Slot0.kD != D) {config.Slot0.kD = D; apply = true;}
        if(config.Slot0.kS != kS) {config.Slot0.kS = kS; apply = true;}
        if(config.MotorOutput.PeakForwardDutyCycle != outputRange) {
            config.MotorOutput.PeakForwardDutyCycle = outputRange;
            config.MotorOutput.PeakReverseDutyCycle = -outputRange;
            apply = true;
        }
        if(apply) applyConfig(); 
        apply = false;
    }

    @Override
    public void setAmpLimit(int limit) {
        if(config.CurrentLimits.StatorCurrentLimit != limit) {
            config.CurrentLimits.StatorCurrentLimitEnable = limit > 0;
            config.CurrentLimits.StatorCurrentLimit = limit;
            applyConfig();
        }
    }

    @Override
    public double getAppliedOutput() {
        return motor.getDutyCycle().getValue(); //TODO: not quite sure if this is right.
    }
}
