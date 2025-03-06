package org.firstinspires.ftc.teamcode.SubSystem;


import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
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
    private final GoBildaPinpointDriver odometer;
    private Pose2D pos;

    public Odometer(HardwareMap hwMap) {
        this.odometer = hwMap.get(GoBildaPinpointDriver.class, "imu");
        this.init();
    }

    private void init(){
        this.odometer.setOffsets(Constants.Odometer.ODOMETER_X_OFFSET, Constants.Odometer.ODOMETER_Y_OFFSET);
        this.odometer.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        this.odometer.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        this.update();
    }

    public double getHeading(){
        return this.pos.getHeading(AngleUnit.DEGREES);
    }

    public double getX(){
        return this.pos.getX(DistanceUnit.MM);
    }

    public double getY(){
        return this.pos.getY(DistanceUnit.MM);
    }

    public void reset(){
        this.odometer.resetPosAndIMU();
    }

    public void update(){
        this.odometer.update();
        this.pos = this.odometer.getPosition();
    }


}
