// package frc.robot.subsystems;

// import com.ctre.phoenix6.hardware.Pigeon2;

// import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
// import static edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.util.Units;
// import edu.wpi.first.wpilibj.Encoder;
// import edu.wpi.first.wpilibj.Notifier;
// import edu.wpi.first.wpilibj.CounterBase.EncodingType;
// import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
// import edu.wpi.first.wpilibj.smartdashboard.Field2d;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants.OdometryConstants;
// import frc.robot.subsystems.CommandSwerveDrivetrain;

// public class DeadWheelSubsystem extends SubsystemBase {

//     // private final CommandSwerveDrivetrain deadWheelGyro;
//     private final Field2d field2dOdo = new Field2d();

//     //Odometry Encoders
//     // private Encoder encoderXdw = new Encoder (8, 9, false, Encoder.EncodingType.k1X);
//     // private Encoder encoderYdw = new Encoder (0, 1, false, Encoder.EncodingType.k1X);
//     private Encoder encoderXdw;
//     private Encoder encoderYdw;

//     //Gyro
//     private Pigeon2 deadWheelGyro = new Pigeon2(25);

//     // private CommandSwerveDrivetrain drivetrainGyro;

//     private OriginPosition originPosition = kBlueAllianceWallRightSide;

//     public DeadWheelSubsystem(){
//         encoderXdw = new Encoder(8, 9, false, EncodingType.k1X);
//         encoderYdw = new Encoder(0, 1, false, EncodingType.k1X);

//         encoderXdw.setDistancePerPulse(Units.inchesToMeters(2 * Math.PI * 1.0 / 360)); //2 * Math.PI * 1.0 / 360
//         encoderYdw.setDistancePerPulse(Units.inchesToMeters(2 * Math.PI * 1.0 / 360)); //2 * Math.PI * 1.0 / 360

//     }

//     @Override
//     public void periodic() {

        
//     }

//     public double getCurrentPositionMetersX() {
//         return encoderXdw.getDistance();
//     }

//     public double getCurrentPositionMetersY() {
//         return encoderYdw.getDistance();
//     }

//     /**
//      * Resets the current pose to the specified pose. This should ONLY be called
//      * when the robot's position on the field is known, like at the beginning of
//      * a match.
//      *
//      * @param newPose new pose                                                                                  
//      */
//     // public void setCurrentPose(Pose2d newPose) {
//     //     deadWheelGyro.seedFieldRelative(newPose);
//     // }

//     /**
//      * Resets the position on the field to 0,0 0-degrees, with forward being
//      * downfield. This resets
//      * what "forward" is for field oriented driving.
//      */
//     // public void resetFieldPosition() {
//     //     setCurrentPose(new Pose2d());
//     // }

    
// }
