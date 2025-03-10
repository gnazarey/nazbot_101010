package org.firstinspires.ftc.teamcode.Teleop;
// FTC Libs
import com.arcrobotics.ftclib.gamepad.GamepadEx;
// Robot Core Libs
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
// FirstInspires Libs
import org.firstinspires.ftc.teamcode.SubSystem.DriveTrain;
import org.firstinspires.ftc.teamcode.SubSystem.Odometer;
import org.firstinspires.ftc.teamcode.SubSystem.RGBLights;
import org.firstinspires.ftc.teamcode.Utils.Constants;
import org.firstinspires.ftc.teamcode.Utils.Logger;
// Java Libraries
import java.util.Locale;

@TeleOp(name="Basic_Op")
public class Basic_Operation extends OpMode {
    private RGBLights lights;
    private Odometer odometer;
    private GamepadEx controller1 = null;
    private DriveTrain driveTrain;
    private Logger logger;
    private double xLeftController;
    private double yLeftController;
    private double xRightController;
    private double yRightController;

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

        this.xRightController = this.controller1.getRightX();
        this.yRightController = this.controller1.getRightY();
        //Select N, S, E, W
        if (this.controller1.getRightX() <= -Constants.Tolerances.CONTROLLER_X) {
            this.driveTrain.setDirection(Constants.DriveBase.Headings.WEST);// west
            this.lights.setlRGB(Constants.RGB_Light.YELLOW);
        } else if (this.controller1.getRightX() >= Constants.Tolerances.CONTROLLER_X) {
            this.driveTrain.setDirection(Constants.DriveBase.Headings.EAST); // east
            this.lights.setrRGB(Constants.RGB_Light.YELLOW);
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
        this.xLeftController = this.controller1.getLeftX();
        this.yLeftController = this.controller1.getLeftY();
        // Drive Control First
        this.driveTrain.move(this.xLeftController,this.yLeftController);

        // Run the Loops
        this.writeLog();
        this.odometer.update();
        this.driveTrain.loop();
        this.lights.setAllRGB(Constants.RGB_Light.WHITE);
    }

    private void writeLog(){
        // JoyStick Input
        this.logger.writeLog(String.format(Locale.ENGLISH,"%s Left Controller X: %f Y: %f",this.logger.getTimeStamp(),this.xLeftController,this.yLeftController));
        this.logger.writeLog(String.format(Locale.ENGLISH,"%s Right Controller X: %f Y: %f",this.logger.getTimeStamp(),this.xRightController,this.yRightController));
    }
    public void stop(){
        this.logger.stopLog();
        this.driveTrain.stop();
    }
}
