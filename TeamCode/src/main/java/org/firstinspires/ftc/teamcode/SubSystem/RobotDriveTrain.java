package org.firstinspires.ftc.teamcode.SubSystem;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Utils.Constants;

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
public class RobotDriveTrain {
    // Motor definition
    private final Motor leftFrontDrive;
    private final Motor leftRearDrive;
    private final Motor rightFrontDrive;
    private final Motor rightRearDrive;
    // Drive Base
    MecanumDrive driveBase;
    // PID Controllers
    private PIDController headingControl = null;
    // Odometer
    private final Odometer odometer;
    // Speed Setting
    private double xSpeed = 0.0;
    private double ySpeed = 0.0;
    private double turnSpeed = 0.0;
    private double maxXSpeed = 1.0;
    private double maxYSpeed = 1.0;
    private double maxTurnSpeed = 1.0;
    // Heading Setting
    private double heading;
    private double headingCorrection = 0.0;
    private double headingSetPoint = Constants.DriveBase.Headings.NORTH;
    private double currentHeading = Constants.DriveBase.Headings.NORTH;
    private Telemetry telemetry;

    public RobotDriveTrain(HardwareMap hwMap){
        // Assign the motors
        this.leftFrontDrive = new Motor(hwMap,"LF");
        this.leftRearDrive = new Motor(hwMap,"LR");
        this.rightFrontDrive = new Motor(hwMap,"RF");
        this.rightRearDrive = new Motor(hwMap,"RR");
        // Assign the drive base
        this.driveBase = new MecanumDrive(this.leftFrontDrive,this.rightFrontDrive,this.leftRearDrive,this.rightRearDrive);
        // PIDController
        this.headingControl = new PIDController(Constants.DriveBase.HEADING_Kp,Constants.DriveBase.HEADING_Ki,Constants.DriveBase.HEADING_Kd);
        // Odometer
        this.odometer = new Odometer(hwMap);
        // Call init
        this.init();
    }

    public void init(){
        // Setup Tolerance
        this.headingControl.setTolerance(Constants.DriveBase.HEADING_TOLERANCE);
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
        // Odometer Update
        this.odometer.reset();
        this.odometer.update();
        // Heading Setting
        this.heading = 0.0;
        // Telemetry
        this.telemetry = FtcDashboard.getInstance().getTelemetry();
    }

    public void loop(){

        this.telemetry.addData("Heading","%.4f",this.odometer.getHeading());
        this.telemetry.addData("X Cord","%.4f",this.odometer.getX());
        this.telemetry.addData("Y Cord","%.4f",this.odometer.getY());
        this.telemetry.addData("Speed mm/s:","%.4f",this.odometer.getTotalSpeed());
        this.telemetry.addData("MPH: ","%.4f",this.odometer.getMPHSpeed());
        this.telemetry.update();
        // Send the movement controls to the controller
        this.driveBase.driveRobotCentric(this.xSpeed,this.ySpeed,this.turnSpeed,false);
        this.odometer.update();
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
        this.currentHeading = newHeading;
    }

    public void setDirection(double newDirection){
        // Check current heading compared to new heading
        this.setHeadingSetPoint(newDirection);
    }

    public void setMaxTurnSpeed(double maxTurn) {
        this.maxTurnSpeed = maxTurn;
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
        this.turnSpeed = 0.0;
        this.driveBase.stop();
    }

    public void turn(double turnDirection){
        // Check turnDirection speed for max
        if ((Math.abs(turnDirection) <= this.maxTurnSpeed)) {
            this.turnSpeed = turnDirection;
        } else if (turnDirection >= 0.0) {
            this.turnSpeed = this.maxTurnSpeed;
        } else {
            this.turnSpeed = -this.maxTurnSpeed;
        }
    }
}
