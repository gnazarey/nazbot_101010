package org.firstinspires.ftc.teamcode.SubSystem;


import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Utils.Constants;
import org.firstinspires.ftc.teamcode.Utils.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.Utils.Logger;

import java.util.Locale;

/////////////////////////////////////////////////
// This is the Odometer subsystem it communicates
// with the odometer and provides functionality
//
// Written By: George Nazarey
// Date: 3/4/2025
// Email: george@nazarey.ca
//
public class Odometer {
    private GoBildaPinpointDriver odometer;
    private Pose2D pos;
    private Pose2D velocity;
    private Logger logger;

    public Odometer(HardwareMap hwMap) {
        this.odometer = hwMap.get(GoBildaPinpointDriver.class, "odometer");
        if (Constants.Odometer.LOGGING) {
            this.logger = new Logger(Constants.Odometer.LOGFILE);
        }
        this.init();
    }

    public Odometer(Logger logger) {
        this.logger = logger;
    }

    private void init(){
        // Setup Odometer
        this.odometer.setOffsets(Constants.Odometer.ODOMETER_X_OFFSET, Constants.Odometer.ODOMETER_Y_OFFSET);
        this.odometer.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        this.odometer.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        // Initialize odometer
        this.reset();
        // Update odometer
        this.update();
    }

    public double getHeading(){ return this.pos.getHeading(AngleUnit.DEGREES); }

    public double getX(){ return this.pos.getX(DistanceUnit.MM); }

    public double getXSpeed() { return this.velocity.getX(DistanceUnit.MM); }

    public double getY() { return this.pos.getY(DistanceUnit.MM); }

    public double getYSpeed() { return this.velocity.getY(DistanceUnit.MM); }

    public void reset(){ this.odometer.resetPosAndIMU(); }

    public void update(){
        this.odometer.update();
        this.pos = this.odometer.getPosition();
        this.velocity = this.odometer.getVelocity();

        if (Constants.Odometer.LOGGING) {
            this.logger.writeLog(String.format(Locale.ENGLISH, "%s Heading: %f", this.logger.getTimeStamp(), this.getHeading()));
            this.logger.writeLog(String.format(Locale.ENGLISH, "%s Position: X: %f Y: %f", this.logger.getTimeStamp(), this.getX(), this.getY()));
            this.logger.writeLog(String.format(Locale.ENGLISH, "%s Velocity: X: %f Y: %f", this.logger.getTimeStamp(), this.getXSpeed(), this.getYSpeed()));
        }
    }


}
