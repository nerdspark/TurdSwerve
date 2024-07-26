// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** This is a sample pod that uses a CANcoder and TalonFXes. */
public class TurdPod extends SubsystemBase {
    public CANcoder absoluteEncoder;
    public TalonFX azimuthMotor;
    public TalonFX driveMotor;
    public double azimuthDriveSpeedMultiplier;
    private double speed = 0;

    TalonFXConfiguration driveConfig = new TalonFXConfiguration();
    TalonFXConfiguration azimuthConfig = new TalonFXConfiguration();
    CANcoderConfiguration coderConfig = new CANcoderConfiguration();

    //variable that determines whether or not to apply PID configurations to the motor (defaults to true for initial application)
    boolean apply = true;

    //making position duty cycle default because it's the simplest. if you want to use a different control type, you can change it
    private final PositionDutyCycle anglePID = new PositionDutyCycle(0).withSlot(0);


    private double initialOffset;


    public TurdPod(int absoluteEncoderID, int azimuthID, int driveID, double absoluteEncoderOffset, boolean azimuthInvert, int azimuthLimit, double azimuthRotationsPerRot, boolean azimuthBrake, double azimuthRR, double kP, double kI, double kD, double FF, double maxOut, double ADMult, boolean driveInvert, int driveLimit, boolean driveBrake, double driveRR) {
        absoluteEncoder = makeCANCoder(absoluteEncoderID, false, absoluteEncoderOffset);
        driveMotor = makeDrive(driveID, driveInvert, driveBrake, driveLimit, driveRR, 1d, 1d);
        
        this.azimuthMotor = new TalonFX(azimuthID);

        //set neutral mode and inverts
        azimuthConfig.MotorOutput.Inverted = azimuthInvert ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        azimuthConfig.MotorOutput.NeutralMode = azimuthBrake ? NeutralModeValue.Brake : NeutralModeValue.Coast;

        //set current limits; supply current limits are hardcoded because they are almost always the same
        azimuthConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        azimuthConfig.CurrentLimits.SupplyCurrentLimit = 40d;
        azimuthConfig.CurrentLimits.SupplyCurrentThreshold = 40d;
        azimuthConfig.CurrentLimits.SupplyTimeThreshold = 100d;

        azimuthConfig.CurrentLimits.StatorCurrentLimitEnable = azimuthLimit > 0;
        azimuthConfig.CurrentLimits.StatorCurrentLimit = azimuthLimit;


        // this is kind of bad code, but it's the easiest way to set a ramp rate regardless of control type
        azimuthConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = azimuthRR;
        azimuthConfig.OpenLoopRamps.TorqueOpenLoopRampPeriod = azimuthRR;
        azimuthConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = azimuthRR; 
        azimuthConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = azimuthRR;
        azimuthConfig.ClosedLoopRamps.TorqueClosedLoopRampPeriod = azimuthRR;
        azimuthConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = azimuthRR;

        //set feedback ratios
        azimuthConfig.Feedback.SensorToMechanismRatio = 1d;
        azimuthConfig.Feedback.RotorToSensorRatio = 1d;

        azimuthConfig.ClosedLoopGeneral.ContinuousWrap = true;
        azimuthConfig.Feedback.FeedbackRemoteSensorID = absoluteEncoderID;
        azimuthConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;

        this.azimuthMotor.getConfigurator().apply(azimuthConfig);

        if(azimuthConfig.Slot0.kP != kP) {azimuthConfig.Slot0.kP = kP; apply = true;}
        if(azimuthConfig.Slot0.kI != kI) {azimuthConfig.Slot0.kI = kI; apply = true;}
        if(azimuthConfig.Slot0.kD != kD) {azimuthConfig.Slot0.kD = kD; apply = true;}
        if(azimuthConfig.Slot0.kS != FF) {azimuthConfig.Slot0.kS = FF; apply = true;}
        if(azimuthConfig.MotorOutput.PeakForwardDutyCycle != maxOut) {
            azimuthConfig.MotorOutput.PeakForwardDutyCycle = maxOut;
            azimuthConfig.MotorOutput.PeakReverseDutyCycle = -maxOut;
            apply = true;
        }
        if(apply) {
            this.azimuthMotor.getConfigurator().apply(azimuthConfig);
        } 
        azimuthDriveSpeedMultiplier = ADMult;

        apply = false;
        
        resetPod();
    }

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
    public TalonFX makeDrive(int id, boolean inverted, boolean isBrake, double statorLimit, double rampRate, double ENCODER_TO_MECHANISM_RATIO, double ROTOR_TO_ENCODER_RATIO) {
        //I figured nobody had the guts to put a CANivore on a turdswerve, so i'm leaving out the CAN bus parameter
        TalonFX motor = new TalonFX(id);

        //set neutral mode and inverts
        driveConfig.MotorOutput.Inverted = inverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        driveConfig.MotorOutput.NeutralMode = isBrake ? NeutralModeValue.Brake : NeutralModeValue.Coast;

        //set current limits; supply current limits are hardcoded because they are almost always the same
        driveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        driveConfig.CurrentLimits.SupplyCurrentLimit = 40d;
        driveConfig.CurrentLimits.SupplyCurrentThreshold = 40d;
        driveConfig.CurrentLimits.SupplyTimeThreshold = 100d;

        driveConfig.CurrentLimits.StatorCurrentLimitEnable = statorLimit > 0;
        driveConfig.CurrentLimits.StatorCurrentLimit = statorLimit;


        // this is kind of bad code, but it's the easiest way to set a ramp rate regardless of control type
        driveConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = rampRate;
        driveConfig.OpenLoopRamps.TorqueOpenLoopRampPeriod = rampRate;
        driveConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = rampRate; 
        driveConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = rampRate;
        driveConfig.ClosedLoopRamps.TorqueClosedLoopRampPeriod = rampRate;
        driveConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = rampRate;

        //set feedback ratios
        driveConfig.Feedback.SensorToMechanismRatio = ENCODER_TO_MECHANISM_RATIO;
        driveConfig.Feedback.RotorToSensorRatio = ROTOR_TO_ENCODER_RATIO;
        //the remote sensor defaults to internal encoder

        motor.getConfigurator().apply(driveConfig);

        return motor;
    }

    /**
     * creates a new CANTurdCoder
     * @param inverted true for CW+, false for CCW+
     * @param offset the offset of the sensor in rotations
     * @param id the CAN id of the sensor
     */
    private CANcoder makeCANCoder(int id, boolean inverted, double offset) {
        CANcoder encoder = new CANcoder(id);

        coderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
        coderConfig.MagnetSensor.SensorDirection = inverted ? SensorDirectionValue.Clockwise_Positive : SensorDirectionValue.CounterClockwise_Positive;
        coderConfig.MagnetSensor.MagnetOffset = -offset;

        //not applying magnet config directly in order to overwrite other settings
        encoder.getConfigurator().apply(coderConfig);
        
        initialOffset = offset;

        return encoder;
    }

    public void setPID(double kS, double P, double I, double D, double outputRange, double ADMult, TalonFX motor) {
        if(azimuthConfig.Slot0.kP != P) {azimuthConfig.Slot0.kP = P; apply = true;}
        if(azimuthConfig.Slot0.kI != I) {azimuthConfig.Slot0.kI = I; apply = true;}
        if(azimuthConfig.Slot0.kD != D) {azimuthConfig.Slot0.kD = D; apply = true;}
        if(azimuthConfig.Slot0.kS != kS) {azimuthConfig.Slot0.kS = kS; apply = true;}
        if(azimuthConfig.MotorOutput.PeakForwardDutyCycle != outputRange) {
            azimuthConfig.MotorOutput.PeakForwardDutyCycle = outputRange;
            azimuthConfig.MotorOutput.PeakReverseDutyCycle = -outputRange;
            apply = true;
        }
        if(apply) {
            motor.getConfigurator().apply(azimuthConfig);
        } 
        azimuthDriveSpeedMultiplier = ADMult;

        apply = false;
    }

    public void setAmpLimit(int limit) {
        if(driveConfig.CurrentLimits.StatorCurrentLimit != limit) {
            driveConfig.CurrentLimits.StatorCurrentLimitEnable = limit > 0;
            driveConfig.CurrentLimits.StatorCurrentLimit = limit;
            driveMotor.getConfigurator().apply(driveConfig);
        }
    }

    public void resetPod() {
        driveMotor.setPosition(0);
        azimuthMotor.setPosition(absoluteEncoder.getAbsolutePosition().getValueAsDouble());
    }

    public void resetZero() {
        absoluteEncoder.setPosition(0);
        resetPod();
    }

    public void revertZero() {
        if(coderConfig.MagnetSensor.MagnetOffset != initialOffset) {
            coderConfig.MagnetSensor.MagnetOffset = initialOffset; 
            absoluteEncoder.getConfigurator().apply(coderConfig);
        }
        resetPod();
    }
    
    public void stop() {
        azimuthMotor.set(0);
        driveMotor.set(0);
    }

    public SwerveModulePosition getPodPosition() {
        return new SwerveModulePosition(driveMotor.getPosition().getValueAsDouble(), Rotation2d.fromRotations(azimuthMotor.getPosition().getValueAsDouble()));
    }

    public void setPodState(SwerveModuleState state) {
        //TODO: for the love of god add comments
        state = SwerveModuleState.optimize(state, Rotation2d.fromRotations(azimuthMotor.getPosition().getValueAsDouble())); // does not account for rotations between 180 and 360?
        azimuthMotor.setControl(anglePID.withPosition(state.angle.getRotations())); 
        speed = Math.abs(state.speedMetersPerSecond) < .01 ? 0 : state.speedMetersPerSecond;
        // SmartDashboard.putNumber("state.angle.getRadians()", state.angle.getRadians());

        double error = (state.angle.getRadians() - absoluteEncoder.getAbsolutePosition().getValueAsDouble()) % (2*Math.PI);
            error = error > Math.PI ? error - 2*Math.PI : error;
            error = error < -Math.PI ? error + 2*Math.PI : error;
            error *= 180 / Math.PI;
    }

    @Override
    public void periodic() {
        driveMotor.set(speed + (azimuthMotor.getDutyCycle().getValue() * azimuthDriveSpeedMultiplier)); //should this be in setPodState?
        
        //TODO: dont use smartdashboard
        SmartDashboard.putNumber("absolute encoder" + absoluteEncoder.getDeviceID(), absoluteEncoder.getAbsolutePosition().getValueAsDouble());
        SmartDashboard.putNumber("azimuth pose " + absoluteEncoder.getDeviceID(), azimuthMotor.getPosition().getValueAsDouble());
        // SmartDashboard.putNumber("azimuth pose " + config.absoluteEncoderID, azimuthMotor.);
        // SmartDashboard.putNumber("drive pos " + driveMotor.getDeviceId(), driveEncoder.getPosition());
        // SmartDashboard.putNumber("azimuth.getAppliedOutput()" + azimuthMotor.getDeviceId(), azimuthMotor.getAppliedOutput()); //getAppliedOutput());
    }
}