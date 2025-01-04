// package frc.robot.utils;

// import java.util.ArrayList;
// import java.util.Arrays;

// // import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.wpilibj.Timer;
// import frc.robot.Constants.OdometryConstants;

// public class MathFunctions {

//     static double robotRotNewOpt = 0.0;
//     static double robotRotOldOpt = 0.0;
//     static double robotRotOpt = 0.0;
//     static double robotRotOptAngle = 0.0;
//     static double robotXdisplacementOpt = 0.0;
//     static double robotYdisplacementOpt = 0.0;
//     static double robotVectorByOdoOpt = 0.0;
//     static double robotVectorMagOpt = 0.0;
//     static double robotFieldAngleOpt = 0.0;
//     static double xPositionOptical = 0.0;
//     static double yPositionOptical = 0.0;
//     static double xDisplacementOld = 0.0;
//     static double yDisplacementOld = 0.0;

//     static double robotGlobalXCoordinatePosition;
//     static double robotGlobalYCoordinatePosition;

//     static double [] deadWheelOdoState = new double[2];

//     private static Timer deadWheelElapsedTime = new Timer();

//     static double prevOdoTime = 0.0;

//     static double robotSpeed = 0.0;
//     static double robotSpeedMAve = 0.0;

//     static ArrayList<Double> MovingAverage3 = new ArrayList<>(Arrays.asList(0.0,0.0,0.0,0.0));
//     static double sumMA3 = 0;
//     static double maAverageSize3 = 6;

//     // static Pose2d deadWhealPose2d = new Pose2d();

//     public static double [] globalCoordinatePositionUpdateNERD (double xPosition, double xSpeed, double yPosition, double ySpeed, double robotAngle) {
// //     public static ArrayList<PointDW> globalCoordinatePositionUpdateNERD (double xPosition, double yPosition, double robotAngle) { 
//         double odoTime = deadWheelElapsedTime.get();
//         double odoLoopTime = odoTime - prevOdoTime;
//         prevOdoTime = odoTime;

//                 //measure encoder position
//         //frontPositionOptical = frontEncoder.getCurrentPosition();
//         // rightPositionOptical = verticalEncoderRight.getCurrentPosition();
//         // leftPositionOptical = verticalEncoderLeft.getCurrentPosition();
//         // backPositionOptical = horizontalEncoder.getCurrentPosition();

//         //First, determine the robot z movement, so encoder ticks caused by z movement can be removed from x, y movement
//         robotRotNewOpt = robotAngle;
// //        robotRotNewOpt = (rightPositionOptical + leftPositionOptical) / (2 * 22.5);
//         robotRotOpt = robotRotNewOpt - robotRotOldOpt;
//         robotRotOldOpt = robotRotNewOpt;
//         robotRotOptAngle += robotRotOpt;

//         //robot rotation (each loop) expressed in motor ticks (robot angle, ticks per degree robot rotation...determined through testing for each encoder wheel).
//         //double robotRotDisplacementOptFront = robotRotOpt * 0; //21.390 ticks per degree of robot rotation
//         double robotRotDisplacementOptX = robotRotOpt * OdometryConstants.DEADWHEEL_TURN_OFFSET_X; //Comp Bot 44.542 Quad 21.085each wheel is mounted slightly different on the bot
//         double robotRotDisplacementOptY = robotRotOpt * OdometryConstants.DEADWHEEL_TURN_OFFSET_Y; //Comp Bot 40.914 Quad 21.318

//         //encoder ticks for each sensor, each loop
//         double xDisplacement = xPosition - xDisplacementOld;
//         double yDisplacement = yPosition - yDisplacementOld;

//         xDisplacementOld = xDisplacement;
//         yDisplacementOld = yDisplacement;

//         //Now, remove the ticks caused by z movement from the total encoder count, each loop
//         double xDispNoRot = xDisplacement - robotRotDisplacementOptX;
//         double yDispNoRot = yDisplacement - robotRotDisplacementOptY;

// //        //This section was created for debugging
// //        leftDispNoRotTotOpt += leftDispNoRot;
// //        rightDispNoRotTotOpt += rightDispNoRot;

//         //calculate X displacement, and convert ticks to inches (2.362 = wheel diameter inches, 1440 = ticks per wheel rot), and account for robot angle and omni wheel effect
// //        robotXdisplacementOpt = rearDispNoRot * ((2.362 * 1.01 * Math.PI) / 8192); //Millenium Ducky Encoders
//         // robotXdisplacementOpt = xDispNoRot * ((2.362 * 1.01 * Math.PI) / 1440); //Quadricycle Encoders
//         robotXdisplacementOpt = xDispNoRot;
//         //calculate Y displacement, and convert ticks to inches (2.362 = wheel diameter inches, 1440 = ticks per wheel rot), and account for robot angle and omni wheel effect
// //        robotYdisplacementOpt = ((rightDispNoRot - leftDispNoRot) / 2) * ((2.362 * 1.01 * Math.PI) / 8192); //Millenium Ducky Encoders
//         // robotYdisplacementOpt = yDispNoRot * ((2.362 * 1.01 * Math.PI) / 1440); //Quadricycle Encoders
//         robotYdisplacementOpt = yDispNoRot;

//         //Using inverse kinematics, calculate the robot driving direction, from the encoder measurements
//         robotVectorByOdoOpt = Math.atan2(robotYdisplacementOpt, robotXdisplacementOpt) * 180 / Math.PI;

//         //Now that we know the robot driving direction, calculate the driving distance, each loop
//         robotVectorMagOpt = Math.sqrt((robotXdisplacementOpt * robotXdisplacementOpt) + (robotYdisplacementOpt * robotYdisplacementOpt));

//         //The calculated robot vector is the direction in field centric.  Adding the robot gyro angle was an error.
//         robotFieldAngleOpt = (robotVectorByOdoOpt + robotAngle);
// //        robotFieldAngleOpt = robotVectorByOdoOpt + robotRotOptAngle;
//         //robotFieldAngleOpt = robotVectorByOdoOpt;

//         //Now we know the driving direction and distance for each loop, use forward kinematics calculation to determine x, y movement, each loop
//         double robotFieldPositionXOpt = robotVectorMagOpt * Math.cos(robotFieldAngleOpt * Math.PI / 180);  //field position in inches
//         double robotFieldPositionYOpt = robotVectorMagOpt * Math.sin(robotFieldAngleOpt * Math.PI / 180);  //field position in inches

//         //Add each x, y loop calculation, to track the robot location on the field
//         robotGlobalXCoordinatePosition += robotFieldPositionXOpt;
//         robotGlobalYCoordinatePosition += robotFieldPositionYOpt;

//         robotSpeed = Math.hypot(robotFieldPositionXOpt, robotFieldPositionYOpt) / odoLoopTime;
//         robotSpeedMAve = MovingAverageCalc3(robotSpeed);

//         deadWheelOdoState [0] = robotGlobalXCoordinatePosition;
//         deadWheelOdoState [1] = robotGlobalYCoordinatePosition;

//         return deadWheelOdoState;

//         // ArrayList<PointDW> allPoints = new ArrayList<>();

//         // allPoints.add(new PointDW(robotGlobalXCoordinatePosition, robotGlobalYCoordinatePosition));

//         // return allPoints;

//     }

//     public static double MovingAverageCalc3(double inputValueMA) {


//         MovingAverage3.add(inputValueMA);

//         if (MovingAverage3.size() > maAverageSize3)
//             MovingAverage3.remove(0);

//         sumMA3 = 0;
//         for(int i = 0; i < MovingAverage3.size(); i++)
//             sumMA3 += MovingAverage3.get(i);

//         return sumMA3/MovingAverage3.size();


//     }
    
// }
