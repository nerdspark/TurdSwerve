// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;

import org.w3c.dom.ls.LSException;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import frc.robot.util.PIDToPosition;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.Odometry;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.DriveToPoseCommand;
import frc.robot.constants.AutoDriveConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Inventory;
import frc.robot.subsystems.PoseEstimatorSubsystem;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    public static final CommandXboxController driverCommand = new CommandXboxController(0);
    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    public static final Inventory inventory = new Inventory();
    private final Telemetry logger = new Telemetry(MaxSpeed);
    public static final XboxController driverRaw = new XboxController(0);
    private final CommandXboxController joystick = new CommandXboxController(0);
    public final PIDToPosition PID = new PIDToPosition();
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final PoseEstimatorSubsystem poseEstimatorSubsystem = new PoseEstimatorSubsystem(drivetrain);
    public RobotContainer() {
        
        //drivetrain.setDefaultCommand(new AutoDriveCommand(drivetrain, driverLeftJoystick, driverRightJoystick, inventory, drive));
        
        configureBindings();
    }

    private void configureBindings() {
        //drivetrain.setDefaultCommand(new AutoDriveCommand(drivetrain, () -> new Translation2d(driverRaw.getRightX(), driverRaw.getRightY()), () -> new Translation2d(driverRaw.getLeftX(), driverRaw.getLeftY()), inventory));
        drivetrain.setDefaultCommand(drivetrain.applyRequest(() ->
            drive.withVelocityX(XMergeCommand() * MaxSpeed*0.1)
                          .withVelocityY(YMergeCommand() * MaxSpeed*0.1)
                          .withRotationalRate(ZMergeCommand())));

        
        //drivetrain.setDefaultCommand(drivetrain.applyRequest(() -> drive.withVelocityX(-AutoDrive().getY() * MaxSpeed*0.1).withVelocityY(-AutoDrive().getX() * MaxSpeed*0.1).withRotationalRate(0.0)));
        // if (AutoDrive().getNorm() > 0.01) {
        //     drivetrain.setDefaultCommand(drivetrain.applyRequest(() -> drive.withVelocityX(-AutoDrive().getY() * MaxSpeed*0.1).withVelocityY(-AutoDrive().getX() * MaxSpeed*0.1).withRotationalRate(0.0)));
        //     SmartDashboard.putBoolean("bool2", true);
        //   } else {
        //     drivetrain.setDefaultCommand(drivetrain.applyRequest(() ->
        //     drive.withVelocityX(-joystick.getLeftY() * MaxSpeed*0.1)
        //                   .withVelocityY(-joystick.getLeftX() * MaxSpeed*0.1)
        //                   .withRotationalRate(-joystick.getRightX() * MaxAngularRate)));
        //                   SmartDashboard.putBoolean("bool2", false);
        // }
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        // drivetrain.setDefaultCommand(
        //     // Drivetrain will execute this command periodically
        //     drivetrain.applyRequest(() ->
        //         drive.withVelocityX(-joystick.getLeftY() * MaxSpeed*0.1) // Drive forward with negative Y (forward)
        //             .withVelocityY(-joystick.getLeftX() * MaxSpeed*0.1) // Drive left with negative X (left)
        //             .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        //     )
        // );

        // joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        // joystick.b().whileTrue(drivetrain.applyRequest(() ->
        //     point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        // ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.a().onTrue(new InstantCommand(() -> inventory.setAstatus(true)));
        // joystick.b().onTrue(new InstantCommand(() -> inventory.setBstatus(true)));
        // joystick.x().onTrue(new InstantCommand(() -> inventory.setXstatus(true)));
        // joystick.y().onTrue(new InstantCommand(() -> inventory.setYstatus(true)));
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
        joystick.rightBumper().whileTrue(new DriveToPoseCommand(drivetrain, () -> poseEstimatorSubsystem.getCurrentPose(), 
            () -> AutoDrive(() -> poseEstimatorSubsystem.getCurrentPose()), ()->drivetrain.getState().Pose.getRotation()).until(() -> joystick.y().getAsBoolean()));
        drivetrain.registerTelemetry(logger::telemeterize);
    }
    public Pose2d AutoDrive(Supplier<Pose2d> robotPose){

        boolean inventoryStatuses = inventory.getAstatus();
        Translation2d BestVector = PID.ChooseVector(robotPose.get(), inventoryStatuses);
        boolean deadzoneA = robotPose.get().getTranslation().getDistance(AutoDriveConstants.position1) < AutoDriveConstants.zone;
        if (deadzoneA == true){
            inventory.setAstatus(false);
        }
        return new Pose2d(BestVector.getX(), BestVector.getY(), new Rotation2d(0.0));
    }
    public double XMergeCommand() {
        
            return -joystick.getLeftY(); 
        
    }
    public double YMergeCommand() {
        
            return -joystick.getLeftX(); 
        
    }
    public double ZMergeCommand() {
        
            return -joystick.getRightX() * MaxAngularRate; 
        
    }
    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
