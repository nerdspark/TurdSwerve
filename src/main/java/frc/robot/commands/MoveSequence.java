// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.TurdSwerve;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class MoveSequence extends SequentialCommandGroup {
  /** Creates a new MoveSequence. */
  public MoveSequence(TurdSwerve swerve) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    SmartDashboard.putBoolean("Started", true);
    addCommands(new TurdDriveAuto(swerve, 0.1, 0).withTimeout(1.8),
                new TurdDriveAuto(swerve, 0.1, 120).withTimeout(3.6),
                new TurdDriveAuto(swerve, 0.1, 240).withTimeout(3.6),
                new TurdDriveAuto(swerve, 0.1, 0).withTimeout(1.8),
                new TurdCease(swerve));
    //addCommands(new TurdDriveAuto(swerve, 0.1, 90).withTimeout(2));
  }
}
