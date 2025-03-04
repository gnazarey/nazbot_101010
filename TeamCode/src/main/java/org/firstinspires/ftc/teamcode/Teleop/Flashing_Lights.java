package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.SubSystem.RGBLights;
import org.firstinspires.ftc.teamcode.Utils.Constants;

@TeleOp(name="FlashingLights")
public class Flashing_Lights extends OpMode {
    private RGBLights lights;
    private boolean flasher = false;

    @Override public void init(){
        this.lights = new RGBLights(hardwareMap);
    }
    @Override public void loop(){
        flasher = !flasher;
        if (flasher){
            this.lights.setrRGB(Constants.RGB_Light.RED);
            this.lights.setlRGB(Constants.RGB_Light.WHITE);
        } else {
            this.lights.setrRGB(Constants.RGB_Light.WHITE);
            this.lights.setlRGB(Constants.RGB_Light.RED);
        }
    }
}
