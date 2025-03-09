package org.firstinspires.ftc.teamcode.Teleop;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.SubSystem.DriveTrain;
import org.firstinspires.ftc.teamcode.SubSystem.Odometer;
import org.firstinspires.ftc.teamcode.SubSystem.RGBLights;
import org.firstinspires.ftc.teamcode.Utils.Constants;
import org.firstinspires.ftc.teamcode.Utils.Logger;

import java.util.Locale;


@TeleOp(name="Basic_Op")
public class Basic_Operation extends OpMode {
    private RGBLights lights;
    private Odometer odometer;
    private GamepadEx controller1 = null;
    private DriveTrain driveTrain;
    private Logger logger;
    private double xController;
    private double yController;

    public void init(){
        this.lights = new RGBLights(hardwareMap);
        this.odometer =new Odometer(hardwareMap);
        this.controller1 = new GamepadEx(gamepad1);
        this.driveTrain = new DriveTrain(hardwareMap);
        // Configure Drive Train
        this.driveTrain.setMaxXSpeed(Constants.DriveBase.MAX_SPEED);
        this.driveTrain.setMaxYSpeed(Constants.DriveBase.MAX_SPEED);
        // Setup Logging
        this.logger = new Logger("basic_operations.csv");
    }

    @Override public void loop(){


        //Select N, S, E, W
        if (this.controller1.getRightX() <= -Constants.Tolerances.CONTROLLER_X) {
            this.driveTrain.setDirection(Constants.DriveBase.Headings.WEST); // west
        } else if (this.controller1.getRightX() >= Constants.Tolerances.CONTROLLER_X) {
            this.driveTrain.setDirection(Constants.DriveBase.Headings.EAST); // east
        } else if (this.controller1.getRightY() >= Constants.Tolerances.CONTROLLER_Y) {
            this.driveTrain.setDirection(Constants.DriveBase.Headings.SOUTH); // south
        } else if (this.controller1.getRightY() <= -Constants.Tolerances.CONTROLLER_Y) {
            this.driveTrain.setDirection(Constants.DriveBase.Headings.NORTH); // north
        }
        // Select Diagonal Directions (NE,SE,SW,NW)
        if (this.controller1.getRightX() > Constants.Tolerances.CONTROLLER_X & this.controller1.getRightY() < -Constants.Tolerances.CONTROLLER_Y) {
            this.driveTrain.setDirection(Constants.DriveBase.Headings.NORTHEAST); // north east
        } else if (this.controller1.getRightX() < -Constants.Tolerances.CONTROLLER_X & this.controller1.getRightY() < -Constants.Tolerances.CONTROLLER_Y) {
            this.driveTrain.setDirection(Constants.DriveBase.Headings.NORTHWEST); // north west
        } else if (this.controller1.getRightX() < -Constants.Tolerances.CONTROLLER_X & this.controller1.getRightY() > Constants.Tolerances.CONTROLLER_Y) {
            this.driveTrain.setDirection(Constants.DriveBase.Headings.SOUTHWEST); // south west
        } else if (this.controller1.getRightX() > Constants.Tolerances.CONTROLLER_X & this.controller1.getRightY() > Constants.Tolerances.CONTROLLER_Y) {
            this.driveTrain.setDirection(Constants.DriveBase.Headings.SOUTHEAST); // south east
        }
        // Look for joystick input
        this.xController = this.controller1.getLeftX();
        this.yController = this.controller1.getLeftY();
        // Drive Control First
        this.driveTrain.move(this.xController,this.yController);
        // Run the Loops
        this.writeLog();
        this.odometer.update();
        this.driveTrain.loop();
    }

    private void writeLog(){
        // JoyStick Input
        this.logger.writeLog(String.format(Locale.ENGLISH,"%s Joystick Input X: %f Y: %f",this.logger.getTimeStamp(),this.xController,this.yController));
    }
    public void stop(){
        this.logger.stopLog();
        this.driveTrain.stop();
    }
}
