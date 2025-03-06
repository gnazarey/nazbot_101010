package org.firstinspires.ftc.teamcode.Teleop;

import android.app.GameManager;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.SubSystem.DriveTrain;
import org.firstinspires.ftc.teamcode.SubSystem.Odometer;
import org.firstinspires.ftc.teamcode.SubSystem.RGBLights;

@TeleOp(name="Basic_Op")
public class Basic_Operation extends OpMode {
    private RGBLights lights;
    private Odometer odometer;
    private GamepadEx driver = null;
    private DriveTrain driveTrain;

    public void init_loop(){

    }

    public void init(){
        this.lights = new RGBLights(hardwareMap);
        this.odometer =new Odometer(hardwareMap);
        this.driver = new GamepadEx(gamepad1);
        this.driveTrain = new DriveTrain(hardwareMap);
    }

    @Override public void loop(){

        // Look for joystick input
        // Drive Control First
        driveTrain.move(driver.getLeftX(),driver.getLeftY());
        // Run the Loops
        driveTrain.loop();
    }
}
