package org.firstinspires.ftc.teamcode.Teleop;
// FTC Libs
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
// Team Libs
import org.firstinspires.ftc.teamcode.SubSystem.Odometer;
import org.firstinspires.ftc.teamcode.SubSystem.RGBLights;
import org.firstinspires.ftc.teamcode.SubSystem.RobotDriveTrain;
import org.firstinspires.ftc.teamcode.Utils.Constants;
import org.firstinspires.ftc.teamcode.Utils.Logger;
// Java Libs
import java.util.Locale;

@TeleOp(name="Wolfie_Drive")
public class Robot_Drive extends OpMode {
    private RGBLights lights;
    private Odometer odometer;
    private GamepadEx controller1 = null;
    private RobotDriveTrain driveTrain;
    private Logger logger;
    private double xLeftController;
    private double yLeftController;
    private double xRightController;
    private double yRightController;

    public void init_loop(){
        this.odometer.reset();
    }

    public void init(){

        this.lights = new RGBLights(hardwareMap);
        this.odometer =new Odometer(hardwareMap);
        this.controller1 = new GamepadEx(gamepad1);
        this.driveTrain = new RobotDriveTrain(hardwareMap);
        // Configure Drive Train
        this.driveTrain.setMaxXSpeed(Constants.DriveBase.MAX_SPEED);
        this.driveTrain.setMaxYSpeed(Constants.DriveBase.MAX_SPEED);
        this.driveTrain.setMaxTurnSpeed(Constants.DriveBase.MAX_SPEED);
        // Setup Logging
        this.logger = new Logger("robotdrive_operations.csv");
    }

    @Override public void loop(){

        // Look for joystick input
        this.xLeftController = this.controller1.getLeftX();
        this.yLeftController = this.controller1.getLeftY();
        this.xRightController = this.controller1.getRightX();
        // Set Blinker
        if (Math.abs(this.xRightController) > 0.0) {
            if (this.xRightController > 0.0){
                this.lights.setrRGB(Constants.RGB_Light.YELLOW);
            } else {
                this.lights.setlRGB(Constants.RGB_Light.YELLOW);
            }
        }
        // Drive Control First
        this.driveTrain.turn(this.xRightController);
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
