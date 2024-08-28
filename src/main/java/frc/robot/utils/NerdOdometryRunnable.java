package frc.robot.utils;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Encoder;


public class NerdOdometryRunnable implements Runnable {
    //Odometry wheels
    private final Encoder encoderX;
    private final Encoder encoderY;

    //Gyro
    private final Pigeon2 odoGyro;

    // private BNO055IMU robotAngle;
    // Orientation lastAngles = new Orientation();
    private double robotAngle = 0.0;
    private double lastRobotAngle = 0.0;
    private double deltaAngle = 0.0;
    private double robotAngleToField = 0.0;

    //Thread run condition
    private boolean isRunning = true;

    //Sleep time interval (milliseconds) for the position udpate thread
    private int sleepTime;

    //NERD Odometry Calculation Parameters
    private double robotRotNewOpt = 0.0;
    private double robotRotOldOpt = 0.0;
    private double robotRotOpt = 0.0;
    // private double robotRotOptAngle = 0.0;
    private double robotVectorByOdoOpt = 0.0;
    private double robotVectorMagOpt = 0.0;
    private double robotFieldAngleOpt = 0.0;
    private double encoderXPosition = 0.0;
    private double encoderYPosition = 0.0;
    private double encoderXPositionOld = 0.0;
    private double encoderYPositionOld = 0.0;
    private double robotGlobalXCoordinatePosition= 0.0;
    private double robotGlobalYCoordinatePosition = 0.0;

    // private ElapsedTime odoElapsedTime = new ElapsedTime();
    private double prevOdoTime = 0.0;
    private double robotSpeed = 0.0;
    private double robotSpeedX = 0.0;
    private double robotSpeedY = 0.0;
    private double robotSpeedZ = 0.0;
    private double robotSpeedMAve = 0.0;
    private double robotSpeedMAveX = 0.0;
    private double robotSpeedMAveY = 0.0;
    private double robotSpeedMAveZ = 0.0;

    /**
     * Constructor for NERD Odometry Thread
     * @param encoderX x direction odometry encoder, facing the horizontal direction
     * @param encoderY y direction odometry encoder, facing the vertical direction
     */

    public NerdOdometryRunnable(Encoder encoderX, Encoder encoderY, Pigeon2 odoGyro, int threadSleepDelay){
        this.encoderX = encoderX;
        this.encoderY = encoderY;
        this.odoGyro = odoGyro;
        sleepTime = threadSleepDelay;
    }

    private synchronized void nerdOdometryUpdate() {

        double odoTime = Utils.getCurrentTimeSeconds();
        double odoLoopTime = odoTime - prevOdoTime;
        double prevOdoTime = odoTime;

        //measure encoder position
        encoderXPosition = encoderX.getDistance();
        encoderYPosition = encoderY.getDistance();

        //First, determine the robot z movement, so encoder ticks caused by z movement can be removed from x, y movement
        robotRotNewOpt = getRobotAngleToField();
        robotRotOpt = robotRotNewOpt - robotRotOldOpt;
        robotRotOldOpt = robotRotNewOpt;
        // robotRotOptAngle += robotRotOpt;

        //robot rotation (each loop) expressed in meters (robot angle, meters per degree robot rotation...determined through testing for each encoder wheel).
        double robotRotDisplacementOptX = robotRotOpt * 22.609; //Comp Bot 44.542 Quad 21.085each wheel is mounted slightly different on the bot
        double robotRotDisplacementOptY = robotRotOpt * 22.376; //Comp Bot 40.914 Quad 21.318

        //encoder ticks for each sensor, each loop
        double xDisplacement = encoderXPosition - encoderXPositionOld;
        double yDisplacement = encoderYPosition - encoderYPositionOld;

        encoderXPositionOld = encoderXPosition;
        encoderXPositionOld = encoderYPosition;

        //Now, remove the ticks caused by z movement from the total encoder count, each loop
        double xDispNoRot = xDisplacement - robotRotDisplacementOptX;
        double yDispNoRot = yDisplacement - robotRotDisplacementOptY;


//        //This section was created for debugging
//        leftDispNoRotTotOpt += leftDispNoRot;
//        rightDispNoRotTotOpt += rightDispNoRot;
//        rearDispNoRotTotOpt += rearDispNoRot;

        //Using inverse kinematics, calculate the robot driving direction, from the encoder measurements
        robotVectorByOdoOpt = Math.atan2(yDispNoRot, xDispNoRot) * 180 / Math.PI;

        //Now that we know the robot driving direction, calculate the driving distance, each loop
        robotVectorMagOpt = Math.sqrt((xDispNoRot * xDispNoRot) + (yDispNoRot * yDispNoRot));

        //The calculated robot vector is the direction in field centric.
        robotFieldAngleOpt = (robotVectorByOdoOpt + getRobotAngleToField());

        //Now we know the driving direction and distance for each loop, use forward kinematics calculation to determine x, y movement, each loop
        double robotFieldPositionXOpt = robotVectorMagOpt * Math.cos(robotFieldAngleOpt * Math.PI / 180);  //field position in inches
        double robotFieldPositionYOpt = robotVectorMagOpt * Math.sin(robotFieldAngleOpt * Math.PI / 180);  //field position in inches

        //Add each x, y loop calculation, to track the robot location on the field
        robotGlobalXCoordinatePosition += robotFieldPositionXOpt;
        robotGlobalYCoordinatePosition += robotFieldPositionYOpt;

        //Calculate Robot Speeds in X, Y, Z, and Combined directions
        robotSpeedX = robotFieldPositionXOpt / odoLoopTime;
        LinearFilter robotSpeedMAveX = LinearFilter.movingAverage(5);
        robotSpeedMAveX.calculate(robotSpeedX);

        robotSpeedY = robotFieldPositionYOpt / odoLoopTime;
        LinearFilter robotSpeedMAveY = LinearFilter.movingAverage(5);
        robotSpeedMAveY.calculate(robotSpeedY);

        robotSpeedZ = robotRotOpt / odoLoopTime;
        LinearFilter robotSpeedMAveZ = LinearFilter.movingAverage(5);
        robotSpeedMAveZ.calculate(robotSpeedZ);

        robotSpeed = Math.hypot(robotFieldPositionXOpt, robotFieldPositionYOpt) / odoLoopTime;
        LinearFilter robotSpeedMAve = LinearFilter.movingAverage(5);
        robotSpeedMAve.calculate(robotSpeed);

    }



    private double getRobotAngleToField() {
        robotAngle = odoGyro.getAngle();

        deltaAngle = robotAngle - lastRobotAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        robotAngle += deltaAngle;

        lastRobotAngle = robotAngle;

        return robotAngleToField;
    }




    /**
     * Returns the robot's global x coordinate
     * @return global x coordinate
     */
    public double returnXCoordinate(){ return robotGlobalXCoordinatePosition; }

    /**
     * Returns the robot's global y coordinate
     * @return global y coordinate
     */
    public double returnYCoordinate(){ return robotGlobalYCoordinatePosition; }

    /**
     * Returns the robot's global orientation
     * @return global orientation, in degrees
     */
    // public double returnOrientation(){ return Math.toDegrees(robotOrientationRadians) % 360; }

    /**
     * Returns the robot's vector heading as observed by Odometry
     * @return global robot vector, in degrees
     */
    public double returnVectorByOdo(){ return robotVectorByOdoOpt; }

    /**
     * Returns the robot's speed as observed by Odometry
     * @return global robot vector, in degrees
     */
    public double returnRobotSpeedMAve(){ return robotSpeedMAve; }

    /**
     * Returns the robot's speed in the X direction as observed by Odometry
     * @return global robot vector, in degrees
     */
    public double returnRobotSpeedMAveX(){ return robotSpeedMAveX; }

    /**
     * Returns the robot's speed in the Y direction as observed by Odometry
     * @return global robot vector, in degrees
     */
    public double returnRobotSpeedMAveY(){ return robotSpeedMAveY; }

        /**
     * Returns the robot's speed in the Z direction as observed by gyro
     * @return global robot vector, in degrees
     */
    public double returnRobotSpeedMAveZ(){ return robotSpeedMAveZ; }

    /**
     * Stops the position update thread
     */
    public void stop(){ isRunning = false; }

    /**
     * Runs the thread
     */
    @Override
    public void run() {
        while(isRunning) {
            nerdOdometryUpdate();
            try {
                Thread.sleep(sleepTime);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }
    
}
