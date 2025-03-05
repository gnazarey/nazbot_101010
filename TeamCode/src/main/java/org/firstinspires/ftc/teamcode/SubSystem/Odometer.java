package org.firstinspires.ftc.teamcode.SubSystem;


import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.Utils.Constants;
import org.firstinspires.ftc.teamcode.Utils.GoBildaPinpointDriver;

/////////////////////////////////////////////////
// This is the Odometer subsystem it communicates
// with the odometer and provides functionality
//
// Written By: George Nazarey
// Date: 3/4/2025
// Email: george@nazarey.ca
//
public class Odometer {
    public GoBildaPinpointDriver odometer;

    public Odometer(HardwareMap hwMap) {

        odometer = hwMap.get(GoBildaPinpointDriver.class, "xy-cord");
        odometer.setOffsets(Constants.Odometer.ODOMETER_X_OFFSET, Constants.Odometer.ODOMETER_Y_OFFSET);
        odometer.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        odometer.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
    }
}
