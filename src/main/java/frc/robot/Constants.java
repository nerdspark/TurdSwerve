package frc.robot;

import edu.wpi.first.math.util.Units;

public final class Constants {

    public final class OdometryConstants {
        
        public static boolean USE_DEADWHEEL = true;

        public static double DEADWHEEL_WHEEL_DIAMETER = Units.inchesToMeters(1.94);  //Diameter of the deadwheel system wheels in inches converted to meters

        public static double DEADWHEEL_TURN_OFFSET_X = 0.002775;  //Amount of x deadwheel encoder drift in meters for one degree of robot rotation
        public static double DEADWHEEL_TURN_OFFSET_Y = 0.002775;  //Amount of y deadwheel encoder drift in meters for one degree of robot rotation
    }
    
}
