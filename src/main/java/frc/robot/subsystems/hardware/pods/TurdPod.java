// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.hardware.pods;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.hardware.encoders.TurdCoder;
import frc.robot.subsystems.hardware.motors.TurdMotor;

public abstract class TurdPod extends SubsystemBase {
    public TurdCoder absoluteEncoder;
    public TurdMotor azimuthMotor;
    public TurdMotor driveMotor;
    public double azimuthDriveSpeedMultiplier;
    public TurdConfig config;
    private double speed = 0;

    //this is very weird but its the only way i could think of doing it.
    /**
     * sets the PID of the azimuth motor
     * @param wildcard dependent on motor type. for TalonFX, this is kS, for SparkMAX, this is IZone
     */
    public void setPID(double wildcard, double P, double I, double D, double outputRange, double ADMult) {
        azimuthMotor.setPID(wildcard, P, I, D, outputRange);
        azimuthDriveSpeedMultiplier = ADMult;
    }
    
    public void setAmpLimit(int ampLimit) {
        driveMotor.setAmpLimit(ampLimit);
    } 

    public void resetPod() {
        driveMotor.setPosition(0);
        azimuthMotor.setPosition(absoluteEncoder.getAbsoluteAngle());
    }

    public void resetZero() {
        absoluteEncoder.setPosition(0);
        resetPod();
    }

    public void revertZero() {
        absoluteEncoder.revertZero();
        resetPod();
    }
    
    public void stop() {
        azimuthMotor.setPower(0);
        driveMotor.setPower(0);
    }

    public SwerveModulePosition getPodPosition() {
        return new SwerveModulePosition(driveMotor.getPosition(), new Rotation2d(azimuthMotor.getPosition()));
    }

    public void setPodState(SwerveModuleState state) {
        //TODO: for the love of god add comments
        state = SwerveModuleState.optimize(state, new Rotation2d(azimuthMotor.getPosition())); // does not account for rotations between 180 and 360?
        azimuthMotor.setTargetPosition(state.angle.getRadians()); 
        speed = Math.abs(state.speedMetersPerSecond) < .01 ? 0 : state.speedMetersPerSecond;
        SmartDashboard.putNumber("state.angle.getRadians()", state.angle.getRadians());

        double error = (state.angle.getRadians() - azimuthMotor.getPosition()) % (2*Math.PI);
            error = error > Math.PI ? error - 2*Math.PI : error;
            error = error < -Math.PI ? error + 2*Math.PI : error;
            error *= 180 / Math.PI;
    }

    public TurdConfig getConfig() {
        return config;
    }


    @Override
    public void periodic() {
        // driveMotor.setPower(speed + (azimuthMotor.getAppliedOutput() * azimuthDriveSpeedMultiplier)); //should this be in setPodState?
        
        //TODO: dont use smartdashboard
        SmartDashboard.putNumber("absolute encoder #" + config.absoluteEncoderID, absoluteEncoder.getAbsoluteAngle());
        SmartDashboard.putNumber("azimuth pose " + config.absoluteEncoderID, azimuthMotor.getPosition() * 2 * Math.PI);
        // SmartDashboard.putNumber("azimuth pose " + config.absoluteEncoderID, azimuthMotor.);

        // SmartDashboard.putNumber("drive pos " + driveMotor.getDeviceId(), driveEncoder.getPosition());
        // SmartDashboard.putNumber("azimuth.getAppliedOutput()" + azimuthMotor.getDeviceId(), azimuthMotor.getAppliedOutput()); //getAppliedOutput());
    }
}
