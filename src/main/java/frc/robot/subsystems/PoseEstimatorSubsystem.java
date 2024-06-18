package frc.robot.subsystems;

import static frc.robot.constants.Constants.thetaStdDevCoefficient;
import static frc.robot.constants.Constants.xyStdDevCoefficient;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.constants.Constants;
import frc.robot.constants.RobotMap;

public class PoseEstimatorSubsystem extends SubsystemBase {

    private final TurdSwerve turdSwerve;

    private final Field2d field2d = new Field2d();
   
    private final SwerveDrivePoseEstimator poseEstimator;

    static {
    }

    public PoseEstimatorSubsystem(TurdSwerve turdSwerve) {
        this.turdSwerve = turdSwerve;
        poseEstimator = new SwerveDrivePoseEstimator(RobotMap.drivetrainKinematics, turdSwerve.getGyro(), turdSwerve.getSwerveModulePosition(), new Pose2d(new Translation2d(), new Rotation2d()));
        LimelightHelpers.setPipelineIndex(Constants.Limelight1, 0);
        LimelightHelpers.setPipelineIndex(Constants.Limelight2, 0);
        LimelightHelpers.setPipelineIndex(Constants.Limelight3, 0);

    
    }
    

    public void addDashboardWidgets(ShuffleboardTab tab) {
        tab.add("Field", field2d).withPosition(0, 0).withSize(6, 4);
        tab.addString("Pose", this::getFomattedPose).withPosition(6, 2).withSize(2, 1);
    }

    @Override
    public void periodic() {
        // Update pose estimator with drivetrain sensors


        // Set the pose on the dashboard
        var dashboardPose = poseEstimator.getEstimatedPosition();
        field2d.setRobotPose(dashboardPose);
        updatePoseEstimates(Constants.Limelight1);
        updatePoseEstimates(Constants.Limelight2);
        updatePoseEstimates(Constants.Limelight3);

        SmartDashboard.putNumber("LL 1 Tagcount results.botpose", LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(Constants.Limelight1).tagCount);
        SmartDashboard.putNumber("LL 2 Tagcount results.botpose", LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(Constants.Limelight2).tagCount);
        SmartDashboard.putNumber("LL 3 Tagcount results.botpose", LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(Constants.Limelight3).tagCount);

        SmartDashboard.putNumber("LL 1 avgTagdist results.botpose", LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(Constants.Limelight1).avgTagDist);
        SmartDashboard.putNumber("LL 2 avgTagDist results.botpose", LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(Constants.Limelight2).avgTagDist);
        SmartDashboard.putNumber("LL 3 avgTagDist results.botpose", LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(Constants.Limelight3).avgTagDist);

        SmartDashboard.putNumber("LL 3 X ", LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(Constants.Limelight3).pose.getX());
        SmartDashboard.putNumber("LL 3 Y ", LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(Constants.Limelight3).pose.getY());
        SmartDashboard.putNumber("LL 3 Rotation ", LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(Constants.Limelight3).pose.getRotation().getDegrees());


    }

    private String getFomattedPose() {
        var pose = getCurrentPose();
        return String.format(
                "(%.3f, %.3f) %.2f degrees",
                pose.getX(), pose.getY(), pose.getRotation().getDegrees());
    }

    public Pose2d getCurrentPose() {
        return poseEstimator.getEstimatedPosition();
    }

    /**
     * Resets the current pose to the specified pose. This should ONLY be called
     * when the robot's position on the field is known, like at the beginning of
     * a match.
     *
     * @param newPose new pose
     */
    // public void setCurrentPose(Pose2d newPose) {
    //     turdSwerve.seedFieldRelative(newPose);
    // }

    /**
     * Resets the position on the field to 0,0 0-degrees, with forward being
     * downfield. This resets
     * what "forward" is for field oriented driving.
     */
    // public void resetFieldPosition() {
    //     setCurrentPose(new Pose2d());
    // }

    /**
     * Calculate the standard deviation of the x and y coordinates.
     *
     * @param poseEstimates The pose estimate
     * @param tagPosesSize The number of detected tag poses
     * @return The standard deviation of the x and y coordinates
     */
    private double calculateXYStdDev(Double avgTagDistance, int tagPosesSize) {
        return xyStdDevCoefficient * Math.pow(avgTagDistance, 2.0) / tagPosesSize;
    }
    /**
     * Calculate the standard deviation of the theta coordinate.
     *
     * @param poseEstimates The pose estimate
     * @param tagPosesSize The number of detected tag poses
     * @return The standard deviation of the theta coordinate
     */
    private double calculateThetaStdDev(Double avgTagDistance, int tagPosesSize) {
        return thetaStdDevCoefficient * Math.pow(avgTagDistance, 2.0) / tagPosesSize;
    }

    /**
     * Updates the inputs for AprilTag vision.
     *
     * @param estimator PhotonVisionRunnable estimator.
     * @param inputs The AprilTagVisionIOInputs object containing the inputs.
     */
    public void updatePoseEstimates(String limelightName) {
        LimelightHelpers.SetRobotOrientation(limelightName, turdSwerve.getGyro().getDegrees(), 0, 0, 0, 0, 0);

        LimelightHelpers.PoseEstimate cameraPose = 
        LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);

        // var results = LimelightHelpers.getLatestResults(limelightName);

        var distanceUsedForCalculatingStdDev = cameraPose.avgTagDist;        
        double timestamp = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName).timestampSeconds;            
        double xyStdDev = calculateXYStdDev(distanceUsedForCalculatingStdDev, cameraPose.tagCount);
        double thetaStdDev =
                    calculateThetaStdDev(distanceUsedForCalculatingStdDev, cameraPose.tagCount);
        if(cameraPose.tagCount != 0 && Math.abs(turdSwerve.getGyroRate()) < 720) {
                poseEstimator.addVisionMeasurement(
                        cameraPose.pose, cameraPose.timestampSeconds, VecBuilder.fill(xyStdDev, xyStdDev, 1));
        }
                    
        }
        
    
}
