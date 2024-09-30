package frc.robot.utils;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import static edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.OdometryConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class NerdOdometrySubsystem extends SubsystemBase {

    private final CommandSwerveDrivetrain driveTrainGyro;
    private final Field2d field2dOdo = new Field2d();

    //Odometry Encoders
    private Encoder encoderX = new Encoder (8, 9, false, Encoder.EncodingType.k4X);
    private Encoder encoderY = new Encoder (0, 1, false, Encoder.EncodingType.k4X);

    //Gyro
    private Pigeon2 odoGyro = new Pigeon2(25);

    // private CommandSwerveDrivetrain drivetrainGyro;

    private static NerdOdometryRunnable deadWheelEstimator;
    private static Notifier odoNotifier;

    private OriginPosition originPosition = kBlueAllianceWallRightSide;
    

    public NerdOdometrySubsystem(CommandSwerveDrivetrain driveTrainGyro){

        this.driveTrainGyro = driveTrainGyro;
        encoderX.setDistancePerPulse(Units.inchesToMeters(2 * Math.PI * 1.0 / 360)); //2 * Math.PI * 1.0 / 360
        encoderY.setDistancePerPulse(Units.inchesToMeters(2 * Math.PI * 1.0 / 360)); //2 * Math.PI * 1.0 / 360

        if (OdometryConstants.USE_DEADWHEEL == true){
            deadWheelEstimator = new NerdOdometryRunnable(encoderX, encoderY, odoGyro, 50);
        

        odoNotifier = new Notifier(() -> deadWheelEstimator.run());
        odoNotifier.setName("runOdo");
        odoNotifier.startPeriodic(0.02);

        }

    }

    public void addDashboardWidgets(ShuffleboardTab tab) {
        tab.add("FieldOdo", field2dOdo).withPosition(0, 0).withSize(6, 4);
        tab.addString("PoseOdo", this::getFomattedPose).withPosition(6, 2).withSize(2, 1);
    }

    @Override
    public void periodic() {
        // Update pose estimator with drivetrain sensors

        if (OdometryConstants.USE_DEADWHEEL == true) {
            updateNerdOdo(deadWheelEstimator);
        } else {
            if (odoNotifier != null) odoNotifier.close();
        }

        // Set the pose on the dashboard
        var dashboardPose = getCurrentPose();
        field2dOdo.setRobotPose(dashboardPose);
    }

    private String getFomattedPose() {
        var pose = getCurrentPose();
        return String.format(
                "(%.3f, %.3f) %.2f degrees",
                pose.getX(), pose.getY(), pose.getRotation().getDegrees());
    }

    public Pose2d getCurrentPose() {
        return driveTrainGyro.getState().Pose;
    }

    /**
     * Resets the current pose to the specified pose. This should ONLY be called
     * when the robot's position on the field is known, like at the beginning of
     * a match.
     *
     * @param newPose new pose                                                                                  
     */
    public void setCurrentPose(Pose2d newPose) {
        driveTrainGyro.seedFieldRelative(newPose);
    }

    /**
     * Resets the position on the field to 0,0 0-degrees, with forward being
     * downfield. This resets
     * what "forward" is for field oriented driving.
     */
    public void resetFieldPosition() {
        setCurrentPose(new Pose2d());
    }

    // @Override
    // public void periodic() {

    //     updateNerdOdo();

    // }

    public void updateNerdOdo(NerdOdometryRunnable nerd){

        var deadWheelX = nerd.returnXCoordinate();
        var deadWheelY = nerd.returnYCoordinate();
        var deadWheelPose = nerd.returnDeadWheelPose();

    };
    
}
