package org.firstinspires.ftc.teamcode.SubSystem;
// Robot Core Libs
import com.qualcomm.robotcore.hardware.HardwareMap;
// FirstInspires Libs
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.Utils.Constants;
import org.firstinspires.ftc.teamcode.Utils.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.Utils.Logger;
// Java Libraries
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
    private Boolean logging = false;

    public Odometer(HardwareMap hwMap, Logger logger) {
        this.odometer = hwMap.get(GoBildaPinpointDriver.class, "odometer");
        if (logger == null){
            if (Constants.Odometer.LOGGING) {
                this.logger = new Logger(Constants.Odometer.LOGFILE);
                this.logging = true;
            } else {
                this.logging = false;
            }
        } else {
            this.logger = logger;
            this.logging = true;
        }
        // Setup Odometer
        this.odometer.setOffsets(Constants.Odometer.ODOMETER_X_OFFSET, Constants.Odometer.ODOMETER_Y_OFFSET);
        this.odometer.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        this.odometer.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        // Initialize odometer
        this.reset();
        // Update odometer
        this.update();
    }
    public Odometer(HardwareMap hwMap) {
        this(hwMap,null);
    }

    public double getHeading(){ return this.pos.getHeading(AngleUnit.DEGREES); }

    public double getRawHeading() { return this.odometer.getHeading(UnnormalizedAngleUnit.DEGREES); }

    public double getMPHSpeed() { return this.getTotalSpeed() / 447.04; }

    public double getTotalSpeed(){ return Math.sqrt(Math.pow(getXSpeed(),2.0)+Math.pow(getYSpeed(),2.0)); }

    public double getX(){ return this.pos.getX(DistanceUnit.MM); }

    public double getXSpeed() { return this.velocity.getX(DistanceUnit.MM); }

    public double getY() { return this.pos.getY(DistanceUnit.MM); }

    public double getYSpeed() { return this.velocity.getY(DistanceUnit.MM); }

    public void reset(){ this.odometer.resetPosAndIMU(); }

    public void update(){
        this.odometer.update();
        this.pos = this.odometer.getPosition();
        this.velocity = this.odometer.getVelocity();

        if (this.logging) {
            this.logger.writeLog(String.format(Locale.ENGLISH, "Heading: %f Raw: %f",this.getHeading(),this.getRawHeading()));
            this.logger.writeLog(String.format(Locale.ENGLISH, "Position: X: %f Y: %f",this.getX(),this.getY()));
            this.logger.writeLog(String.format(Locale.ENGLISH, "Velocity: X: %f Y: %f",this.getXSpeed(),this.getYSpeed()));
        }
    }


}
