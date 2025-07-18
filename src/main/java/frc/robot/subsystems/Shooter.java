// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  private final SparkMax intake;
  private final SparkMax flyWheel;
  /** Creates a new Shooter. */
  public Shooter() {
    intake = new SparkMax(Constants.Intake.IntakeID, MotorType.kBrushless);
    flyWheel = new SparkMax(Constants.FlyWheel.flyWheelID, MotorType.kBrushless);
    SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(IdleMode.kBrake);
        
        config.encoder.positionConversionFactor(360.0 / Constants.Intake.gearRatio)
        .velocityConversionFactor(360.0 / Constants.Intake.gearRatio / 60.0);
    intake.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    SparkMaxConfig flyWheelConfig = new SparkMaxConfig();
        config.idleMode(IdleMode.kBrake);
        
        config.encoder.positionConversionFactor(360.0 / Constants.FlyWheel.gearRatio)
        .velocityConversionFactor(360.0 / Constants.FlyWheel.gearRatio / 60.0);
    intake.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }
  public void setPowerIntake(double power){
    intake.set(power);
  }
  public void setPowerFlyWheel(double power){
    flyWheel.set(power);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
