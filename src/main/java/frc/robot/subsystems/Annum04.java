// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.subsystems.LimeLight;
import frc.robot.subsystems.Gyro;
import frc.robot.generated.TunerConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Annum04 extends SubsystemBase {
  private final PIDController follow;
  private final PIDController turn;
  private final LimeLight ll;
  private final Gyro gyro;

  static final double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps;
  static final double MaxAngularRate = TunerConstants.kSpeedAt12VoltsMps * Math.PI;

  private static final double kP_follow = 0.5;
  private static final double kI_follow = 0.00;
  private static final double kD_follow = 0.00;

  private static final double kP_turn = 0.007;
  private static final double kI_turn = 0.02;
  private static final double kD_turn = 0.2;

  double va;
  double vf;
  double vx;
  double vy;

  public Annum04(LimeLight ll, Gyro gyro) {
    this.ll = ll;
    this.gyro = gyro;
    this.follow = new PIDController(kP_follow, kI_follow, kD_follow);
    this.turn = new PIDController(kP_turn, kI_turn, kD_turn);
    ll.setPipelineNumber(0);
    turn.setTolerance(0.25);
    turn.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void periodic() {
    va = 0.0;
    vf = 0.0;
    vx = 0.0;
    vy = 0.0;

    double tx = ll.getTx();
    double ty = ll.getTy();
    double tid = ll.getID();
    
    double a = gyro.getGyro().getRadians();

    if (ll.hasTarget()) {
      va = turn.calculate(tx, 0.0);
      vf = follow.calculate(ty, 0.0);

      vx = Math.sin(a) * vf;
      vy = -Math.cos(a) * vf;

    } else {
      va = turn.calculate(a,0);

      vx = 0.0;
      vy = 0.0;
    }

    SmartDashboard.putBoolean("Detected?", ll.hasTarget());
    SmartDashboard.putNumber("ID:", tid);
    SmartDashboard.putNumber("vx", vx);
    SmartDashboard.putNumber("vy", vy);
    SmartDashboard.putNumber("va", va);
    SmartDashboard.putNumber("gyro", gyro.getGyro().getDegrees());
  }

  public double getVX() {
    return vx;
  }

  public double getVY() {
    return vy;
  }

  public double getVA() {
    return va;
  }
}
