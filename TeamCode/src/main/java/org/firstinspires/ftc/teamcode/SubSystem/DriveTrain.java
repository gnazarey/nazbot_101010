package org.firstinspires.ftc.teamcode.SubSystem;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Utils.Constants;
import org.firstinspires.ftc.teamcode.Utils.Logger;

import java.util.Locale;

/////////////////////////////////////////////////
// This is the DriveTrain subsystem. It contains
// all of the initialization and functionality
// to control the motors
//
// Written By: George Nazarey
// Date: 3/4/2025
// Email: george@nazarey.ca
//
@Config
public class DriveTrain {
    // Hardware Mapping
    private final HardwareMap hwMap;
    // Motor definition
    private Motor leftFrontDrive;
    private Motor leftRearDrive;
    private Motor rightFrontDrive;
    private Motor rightRearDrive;
    // Drive Base
    MecanumDrive driveBase;
    // PID Controllers
    private final PIDController headingControl;
    public static PIDCoefficients headingPID = new PIDCoefficients(Constants.DriveBase.HEADING_Kp,Constants.DriveBase.HEADING_Ki,Constants.DriveBase.HEADING_Kd);
    // Odometer
    private final Odometer odometer;
    // Speed Setting
    private double xSpeed = 0.0;
    private double ySpeed = 0.0;
    private double maxXSpeed = 1.0;
    private double maxYSpeed = 1.0;
    // Heading Setting
    private double heading = 0.0;
    private double headingCorrection = 0.0;
    private double headingSetPoint = Constants.DriveBase.Headings.NORTH;
    public static double headingTolerance = Constants.DriveBase.HEADING_TOLERANCE;
    // Telemetry
    private final boolean useTelemetry;
    private Telemetry telemetry;
    // Looping
    private Logger logger;
    private final boolean logging;

    public DriveTrain(HardwareMap hwMap){
        this(hwMap,null,null,null);
    }

    public DriveTrain(HardwareMap hwMap, Logger logger){
        this(hwMap,logger,null,null);
    }

    public DriveTrain(HardwareMap hwMap, Logger logger, Odometer odo){
        this(hwMap,logger,odo,null);
    }

    public DriveTrain(HardwareMap hwMap, Logger logger, Odometer odo, Telemetry tel){
        // Hardware mapping
        this.hwMap = hwMap;
        // Telemetry
        if ( tel == null){
            this.useTelemetry = false;
        } else {
            this.useTelemetry = true;
            this.telemetry = tel;
        }
        // Logging
        if ( logger == null){
            this.logging = false;
        } else {
            this.logger = logger;
            this.logging = true;
        }
        // Odometer
        if ( odo == null){
            this.odometer = new Odometer(this.hwMap);
        } else {
            this.odometer = odo;
        }
        this.odometer.reset();
        this.odometer.update();
        //  Motors
        this.assignMotors();
        this.initializeMotors();
        // Assign the drive base
        this.driveBase = new MecanumDrive(this.leftFrontDrive,this.rightFrontDrive,this.leftRearDrive,this.rightRearDrive);
        // PIDController
        this.headingControl = new PIDController(DriveTrain.headingPID.p,DriveTrain.headingPID.i,DriveTrain.headingPID.d);
        this.headingControl.setTolerance(DriveTrain.headingTolerance);
        if (logging) {
            this.logger.writeLog(String.format(Locale.ENGLISH, "Max Speed Setting X: %f Y: %f", this.getMaxXSpeed(), this.getMaxYSpeed()));
            this.logger.writeLog(String.format(Locale.ENGLISH, "Heading Kp = %f,Heading Ki = %f,Heading Kd = %f", DriveTrain.headingPID.p,DriveTrain.headingPID.i,DriveTrain.headingPID.d));
        }
    }

    public void loop(){
        // Heading calculations
        this.headingControl.setSetPoint(this.headingSetPoint);
        this.heading = this.odometer.getHeading();
        if (!this.headingControl.atSetPoint()) {
            this.headingCorrection = -this.headingControl.calculate(this.heading);
        } else {
            this.headingCorrection = 0.0;
        }
     //   this.headingCorrection = this.turnCorrection * this.headingCorrection;
        if (this.useTelemetry) {
            this.telemetry.addData("Heading", this.heading);
            this.telemetry.addData("Heading Correction", this.headingCorrection);
            this.telemetry.update();
        }
        if (this.logging){
            this.logger.writeLog(String.format(Locale.ENGLISH,"xPos - yPos - xSpeed - ySpeed - Heading Correction - Heading - HeadingSetPoint),%f,%f,%f,%f,%f,%f,%f",this.odometer.getX(),this.odometer.getY(),xSpeed,this.ySpeed,this.getHeadingCorrection(),this.heading,this.getHeadingSetPoint()));
        }
        // Send the movement controls to the controller
        // xSpeed is the strafing speed -1.0 to +1.0;
        // ySpeed is the forward speed -1.0 to +1.0;
        // headingCorrection speed to turn at -1.0 to +1.0;
        // heading the direction it is facing 0 - 360;
        this.driveBase.driveFieldCentric(this.xSpeed,this.ySpeed,this.headingCorrection,this.heading,false);
    }

    public double getHeadingCorrection(){
        return this.headingCorrection;
    }

    public double getHeadingSetPoint(){
        return this.headingSetPoint;
    }

    public double getMaxXSpeed(){
        return this.maxXSpeed;
    }

    public double getMaxYSpeed(){
        return this.maxYSpeed;
    }

    public void move(double xDirection, double yDirection){
        // Check xDirection speed for max
        if ((Math.abs(xDirection) <= this.maxXSpeed)) {
            this.xSpeed = xDirection;
        } else if (xDirection >= 0.0) {
            this.xSpeed = this.maxXSpeed;
        } else {
            this.xSpeed = -this.maxXSpeed;
        }
        // Check yDirection speed for max
        if ((Math.abs(yDirection) <= this.maxYSpeed)) {
            this.ySpeed = yDirection;
        } else if (yDirection >= 0.0) {
            this.ySpeed = this.maxYSpeed;
        } else {
            this.ySpeed = -this.maxYSpeed;
        }
    }

    public void setHeadingSetPoint(double newHeading){
        this.headingSetPoint = newHeading;
    }

    public void setDirection(double newDirection){
        // Check current heading compared to new heading
        this.setHeadingSetPoint(newDirection);
    }

    public void setMaxXSpeed(double maxX){
        this.maxXSpeed = maxX;
    }

    public void setMaxYSpeed(double maxY){
        this.maxYSpeed = maxY;
    }

    public void stop(){
        this.xSpeed = 0.0;
        this.ySpeed = 0.0;
        this.driveBase.stop();
    }

    private void assignMotors(){
        // Assign the motors
        this.leftFrontDrive = new Motor(this.hwMap,"LF");
        this.leftRearDrive = new Motor(this.hwMap,"LR");
        this.rightFrontDrive = new Motor(this.hwMap,"RF");
        this.rightRearDrive = new Motor(this.hwMap,"RR");
    }

    private void initializeMotors(){
        // Initialize Motors
        this.leftFrontDrive.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        this.leftRearDrive.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        this.rightFrontDrive.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        this.rightRearDrive.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        // Inverting, since it isn't direct drive
        this.leftFrontDrive.setInverted(true);
        this.leftRearDrive.setInverted(true);
        this.rightFrontDrive.setInverted(true);
        this.rightRearDrive.setInverted(true);
    }
}
