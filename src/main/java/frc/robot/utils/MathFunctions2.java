package frc.robot.utils;

import java.util.ArrayList;
import java.util.Arrays;

// import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.OdometryConstants;

public class MathFunctions2 {

        static double [] deadWheelOdoState = new double[2];


    public static double [] globalCoordinatePositionUpdateNERD (double xEncoderDist, double xEncoderDistOld, double xSpeed, double yEncoderDist, 
        double yEncoderDistOld, double ySpeed, double robotAngle, double robotAngleOld, double odoLoopTimer, double odoLoopTimeOld, 
        double robotGlobalXCoordinatePosition, double robotGlobalYCoordinatePosition) {

        // double odoLoopTime = (odoLoopTimer - odoLoopTimeOld) * 1000;

        double xEncoderDelta = xEncoderDist - xEncoderDistOld;
        double yEncoderDelta = yEncoderDist - yEncoderDistOld;

        double robotAngleDelta = -robotAngle + robotAngleOld;

        double robotRotDisplacementOptX = robotAngleDelta * OdometryConstants.DEADWHEEL_TURN_OFFSET_X; 
        double robotRotDisplacementOptY = robotAngleDelta * OdometryConstants.DEADWHEEL_TURN_OFFSET_Y; 

        double xDisplacement = xEncoderDelta - robotRotDisplacementOptX;
        double yDisplacement = yEncoderDelta - robotRotDisplacementOptY;

        //Using inverse kinematics, calculate the robot driving direction, from the encoder measurements
        double robotVectorByOdoOpt = Math.atan2(yDisplacement, xDisplacement);

        //Now that we know the robot driving direction, calculate the driving distance, each loop
        double robotVectorMagOpt = Math.sqrt((xDisplacement * xDisplacement) + (yDisplacement * yDisplacement));

        //The calculated robot vector is the direction in field centric.  Adding the robot gyro angle was an error.
        double robotFieldAngleOpt = (robotVectorByOdoOpt + robotAngle);

        //Now we know the driving direction and distance for each loop, use forward kinematics calculation to determine x, y movement, each loop
        double robotFieldPositionXOpt = robotVectorMagOpt * Math.cos(robotFieldAngleOpt);  //field position in inches
        double robotFieldPositionYOpt = robotVectorMagOpt * Math.sin(robotFieldAngleOpt);  //field position in inches

        // //Add each x, y loop calculation, to track the robot location on the field
        // robotGlobalXCoordinatePosition += robotFieldPositionXOpt;
        // robotGlobalYCoordinatePosition += robotFieldPositionYOpt;

        deadWheelOdoState [0] = robotGlobalXCoordinatePosition + robotFieldPositionXOpt;
        deadWheelOdoState [1] = robotGlobalYCoordinatePosition + robotFieldPositionYOpt;

        return deadWheelOdoState;
    }

//     public static double MovingAverageCalc3(double inputValueMA) {


//         MovingAverage3.add(inputValueMA);

//         if (MovingAverage3.size() > maAverageSize3)
//             MovingAverage3.remove(0);

//         sumMA3 = 0;
//         for(int i = 0; i < MovingAverage3.size(); i++)
//             sumMA3 += MovingAverage3.get(i);

//         return sumMA3/MovingAverage3.size();


//     }
    
}
