package org.firstinspires.ftc.teamcode.Utils;

/////////////////////////////////////////////////
// This is the constant package for Nazbot
// This class will include subclass for each of
// the components. They will be separated to keep
// the project easier to manage.
//
// Written By: George Nazarey
// Date: 2/27/2025
// Email: george@nazarey.ca
//

public class Constants {

    // This is the auto component of the class
    public static final class Auto {
        public static final double MAX_SPEED = 0.3;
        public static final double XPID_Kp = 0.018;
        public static final double YPID_Kp = 0.018;
        public static final double HEADING_Kp = 0.015;
    }
    // This is the odometer component of the class
    public static final class Odometer{
        public static double ODOMETER_X_OFFSET = 0;// uppdated after contact with gobilda support
        public static double ODOMETER_Y_OFFSET = 21.0;
    }
    // This is the rgb_light component of the class
    public static final class RGB_Light {
        public static final double OFF = 0.0;
        public static final double BLACK = 0.0;
        public static final double RED = 0.279;
        public static final double ORANGE = 0.333;
        public static final double YELLOW = 0.388;
        public static final double SAGE = 0.444;
        public static final double GREEN = 0.5;
        public static final double AZURE = 0.555;
        public static final double BLUE =  0.611;
        public static final double INDIGO = 0.666;
        public static final double VIOLET = 0.722;
        public static final double WHITE = 1.000;
    // This is the servercontroller component of the class
    public static final class ServerController {
        public static final int LEFT_REAR_RGB = 0;
        public static final int RIGHT_REAR_RGB = 1;
    }
    }
}
