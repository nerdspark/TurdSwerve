// package frc.robot.utils;

// import java.io.File;

// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.wpilibj.Encoder;


// public class nerdOdometry implements Runnable {
//       //Odometry wheels
//     private Encoder encoderX = new Encoder (8, 9, false, Encoder.EncodingType.k4X);
//     private Encoder encoderY = new Encoder (0, 1, false, Encoder.EncodingType.k4X);

//     encoderX.setDistancePerPulse(Math.inchesToMeters(2 * Math.PI * 1.0 / 360));

//     // private BNO055IMU robotAngle;
//     Orientation lastAngles = new Orientation();
//     private double robotAngleToField = 0;

//     //Thead run condition
//     private boolean isRunning = true;

//     //Position variables used for storage and calculations
//     double verticalRightEncoderWheelPosition = 0, verticalLeftEncoderWheelPosition = 0, normalEncoderWheelPosition = 0,  changeInRobotOrientation = 0;
//     private double robotGlobalXCoordinatePosition = 0, robotGlobalYCoordinatePosition = 0, robotOrientationRadians = 0;
//     private double previousVerticalRightEncoderWheelPosition = 0, previousVerticalLeftEncoderWheelPosition = 0, prevNormalEncoderWheelPosition = 0;

//     //Algorithm constants
//     private double robotEncoderWheelDistance;
//     private double horizontalEncoderTickPerDegreeOffset;

//     //Sleep time interval (milliseconds) for the position update thread
//     private int sleepTime;

//     //Files to access the algorithm constants
//     private File wheelBaseSeparationFile = AppUtil.getInstance().getSettingsFile("wheelBaseSeparation.txt");
//     private File horizontalTickOffsetFile = AppUtil.getInstance().getSettingsFile("horizontalTickOffset.txt");

//     private int verticalLeftEncoderPositionMultiplier = 1;
//     private int verticalRightEncoderPositionMultiplier = 1;
//     private int normalEncoderPositionMultiplier = 1;

//     //NERD Odometry Calculation Parameters
//     double robotRotNewOpt = 0;
//     double robotRotOldOpt = 0;
//     double robotRotOpt = 0;
//     double robotRotOptAngle = 0;
//     double robotXdisplacementOpt = 0;
//     double robotYdisplacementOpt = 0;
//     double robotVectorByOdoOpt = 0;
//     double robotVectorMagOpt = 0;
//     double robotFieldAngleOpt = 0;
//     double rightPositionOptical = 0;
//     double leftPositionOptical = 0;
//     double backPositionOptical = 0;
//     double leftDisplacementOld = 0;
//     double rightDisplacementOld = 0;
//     double rearDisplacementOld = 0;
//     double xPositionOpt = 0;
//     double yPositionOpt = 0;

//     private ElapsedTime odoElapsedTime = new ElapsedTime();
//     double prevOdoTime = 0;
//     double robotSpeed = 0;
//     double robotSpeedMAve = 0;

//     /**
//      * Constructor for GlobalCoordinatePosition Thread
//      * @param verticalEncoderLeft left odometry encoder, facing the vertical direction
//      * @param verticalEncoderRight right odometry encoder, facing the vertical direction
//      * @param horizontalEncoder horizontal odometry encoder, perpendicular to the other two odometry encoder wheels
//      * @param threadSleepDelay delay in milliseconds for the GlobalPositionUpdate thread (50-75 milliseconds is suggested)
//      */
//     public nerdOdometry(Encoder encoderX, Encoder encoderY, Pose2d robotPose, int threadSleepDelay){
//         this.encoderX = encoderX;
//         this.encoderY = encoderY;
//         sleepTime = threadSleepDelay;
//         this.robotAngle = imu;


//         robotEncoderWheelDistance = Double.parseDouble(ReadWriteFile.readFile(wheelBaseSeparationFile).trim()) * COUNTS_PER_INCH;
//         this.horizontalEncoderTickPerDegreeOffset = Double.parseDouble(ReadWriteFile.readFile(horizontalTickOffsetFile).trim());

//     }

//     private synchronized void nerdOdometryUpdate() {

//         double odoTime = odoElapsedTime.seconds();
//         double odoLoopTime = odoTime - prevOdoTime;
//         prevOdoTime = odoTime;

//         //measure encoder position
//         //frontPositionOptical = frontEncoder.getCurrentPosition();
//         rightPositionOptical = verticalEncoderRight.getCurrentPosition();
//         leftPositionOptical = verticalEncoderLeft.getCurrentPosition();
//         backPositionOptical = horizontalEncoder.getCurrentPosition();

//         //First, determine the robot z movement, so encoder ticks caused by z movement can be removed from x, y movement
//         robotRotNewOpt = getAngle();
// //        robotRotNewOpt = (rightPositionOptical + leftPositionOptical) / (2 * 22.5);
//         robotRotOpt = robotRotNewOpt - robotRotOldOpt;
//         robotRotOldOpt = robotRotNewOpt;
//         robotRotOptAngle += robotRotOpt;

//         //robot rotation (each loop) expressed in motor ticks (robot angle, ticks per degree robot rotation...determined through testing for each encoder wheel).
//         //double robotRotDisplacementOptFront = robotRotOpt * 0; //21.390 ticks per degree of robot rotation
//         double robotRotDisplacementOptRight = robotRotOpt * 22.609; //Comp Bot 44.542 Quad 21.085each wheel is mounted slightly different on the bot
//         double robotRotDisplacementOptLeft = robotRotOpt * 22.376; //Comp Bot 40.914 Quad 21.318
//         double robotRotDisplacementOptBack = robotRotOpt * 22.436; //Comp Bot 11.861, Quad 21.093

//         //encoder ticks for each sensor, each loop
//         double leftDisplacement = leftPositionOptical - leftDisplacementOld;
//         double rightDisplacement = rightPositionOptical - rightDisplacementOld;
//         double rearDisplacement = backPositionOptical - rearDisplacementOld;

//         leftDisplacementOld = leftPositionOptical;
//         rightDisplacementOld = rightPositionOptical;
//         rearDisplacementOld = backPositionOptical;

//         //Now, remove the ticks caused by z movement from the total encoder count, each loop
//         double leftDispNoRot = leftDisplacement - robotRotDisplacementOptRight;
//         double rightDispNoRot = rightDisplacement - robotRotDisplacementOptLeft;
//         double rearDispNoRot = rearDisplacement - robotRotDisplacementOptBack;

// //        //This section was created for debugging
// //        leftDispNoRotTotOpt += leftDispNoRot;
// //        rightDispNoRotTotOpt += rightDispNoRot;
// //        rearDispNoRotTotOpt += rearDispNoRot;

//         //calculate X displacement, and convert ticks to inches (2.362 = wheel diameter inches, 1440 = ticks per wheel rot), and account for robot angle and omni wheel effect
// //        robotXdisplacementOpt = rearDispNoRot * ((2.362 * 1.01 * Math.PI) / 8192); //Millenium Ducky Encoders
//         robotXdisplacementOpt = rearDispNoRot * ((2.362 * 1.01 * Math.PI) / 1440); //Quadricycle Encoders
//         //calculate Y displacement, and convert ticks to inches (2.362 = wheel diameter inches, 1440 = ticks per wheel rot), and account for robot angle and omni wheel effect
// //        robotYdisplacementOpt = ((rightDispNoRot - leftDispNoRot) / 2) * ((2.362 * 1.01 * Math.PI) / 8192); //Millenium Ducky Encoders
//         robotYdisplacementOpt = ((rightDispNoRot - leftDispNoRot) / 2) * ((2.362 * 1.01 * Math.PI) / 1440); //Quadricycle Encoders

//         //Using inverse kinematics, calculate the robot driving direction, from the encoder measurements
//         robotVectorByOdoOpt = Math.atan2(robotYdisplacementOpt, robotXdisplacementOpt) * 180 / Math.PI;

//         //Now that we know the robot driving direction, calculate the driving distance, each loop
//         robotVectorMagOpt = Math.sqrt((robotXdisplacementOpt * robotXdisplacementOpt) + (robotYdisplacementOpt * robotYdisplacementOpt));

//         //The calculated robot vector is the direction in field centric.  Adding the robot gyro angle was an error.
//         robotFieldAngleOpt = (robotVectorByOdoOpt + getAngle());
// //        robotFieldAngleOpt = robotVectorByOdoOpt + robotRotOptAngle;
//         //robotFieldAngleOpt = robotVectorByOdoOpt;

//         //Now we know the driving direction and distance for each loop, use forward kinematics calculation to determine x, y movement, each loop
//         double robotFieldPositionXOpt = robotVectorMagOpt * Math.cos(robotFieldAngleOpt * Math.PI / 180);  //field position in inches
//         double robotFieldPositionYOpt = robotVectorMagOpt * Math.sin(robotFieldAngleOpt * Math.PI / 180);  //field position in inches

//         //Add each x, y loop calculation, to track the robot location on the field
//         robotGlobalXCoordinatePosition += robotFieldPositionXOpt;
//         robotGlobalYCoordinatePosition += robotFieldPositionYOpt;

//         robotSpeed = Math.hypot(robotFieldPositionXOpt, robotFieldPositionYOpt) / odoLoopTime;
//         robotSpeedMAve = MathFunctions3.MovingAverageCalc3(robotSpeed);

//     }



//     private double getAngle() {

//         Orientation angles = robotAngle.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

//         double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

//         if (deltaAngle < -180)
//             deltaAngle += 360;
//         else if (deltaAngle > 180)
//             deltaAngle -= 360;

//         robotAngleToField += deltaAngle;

//         lastAngles = angles;

//         return robotAngleToField;
//     }




//     /**
//      * Returns the robot's global x coordinate
//      * @return global x coordinate
//      */
//     public double returnXCoordinate(){ return robotGlobalXCoordinatePosition; }

//     /**
//      * Returns the robot's global y coordinate
//      * @return global y coordinate
//      */
//     public double returnYCoordinate(){ return robotGlobalYCoordinatePosition; }

//     /**
//      * Returns the robot's global orientation
//      * @return global orientation, in degrees
//      */
//     public double returnOrientation(){ return Math.toDegrees(robotOrientationRadians) % 360; }

//     /**
//      * Returns the robot's vector heading as observed by Odometry
//      * @return global robot vector, in degrees
//      */
//     public double returnVectorByOdo(){ return robotVectorByOdoOpt; }

//     /**
//      * Returns the robot's vector heading as observed by Odometry
//      * @return global robot vector, in degrees
//      */
//     public double returnRobotSpeedMAve(){ return robotSpeedMAve; }

//     /**
//      * Stops the position update thread
//      */
//     public void stop(){ isRunning = false; }

//     public void reverseLeftEncoder(){
//         if(verticalLeftEncoderPositionMultiplier == 1){
//             verticalLeftEncoderPositionMultiplier = -1;
//         }else{
//             verticalLeftEncoderPositionMultiplier = 1;
//         }
//     }

//     public void reverseRightEncoder(){
//         if(verticalRightEncoderPositionMultiplier == 1){
//             verticalRightEncoderPositionMultiplier = -1;
//         }else{
//             verticalRightEncoderPositionMultiplier = 1;
//         }
//     }

//     public void reverseNormalEncoder(){
//         if(normalEncoderPositionMultiplier == 1){
//             normalEncoderPositionMultiplier = -1;
//         }else{
//             normalEncoderPositionMultiplier = 1;
//         }
//     }

//     /**
//      * Runs the thread
//      */
//     @Override
//     public void run() {
//         while(isRunning) {
//             nerdOdometryUpdate();
//             try {
//                 Thread.sleep(sleepTime);
//             } catch (InterruptedException e) {
//                 e.printStackTrace();
//             }
//         }
//     }
    
// }
