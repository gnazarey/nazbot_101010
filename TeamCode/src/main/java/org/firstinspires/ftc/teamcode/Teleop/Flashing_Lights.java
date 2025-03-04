package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.SubSystem.RGBLights;
import org.firstinspires.ftc.teamcode.Utils.Constants;

@TeleOp(name="FlashingLights")
public class Flashing_Lights extends OpMode {
    private RGBLights lights = null;

    @Override public void init(){
        this.lights = new RGBLights(hardwareMap);

}
    @Override public void loop(){
        this.lights.setlRGB(Constants.RGB_Light.AZURE);
        this.lights.setrRGB(Constants.RGB_Light.BLUE);
        this.lights.setlRGB(Constants.RGB_Light.RED);
        this.lights.setrRGB(Constants.RGB_Light.GREEN);
        this.lights.setlRGB(Constants.RGB_Light.INDIGO);
        this.lights.setrRGB(Constants.RGB_Light.ORANGE);
        this.lights.setlRGB(Constants.RGB_Light.VIOLET);
        this.lights.setrRGB(Constants.RGB_Light.SAGE);
}
}
