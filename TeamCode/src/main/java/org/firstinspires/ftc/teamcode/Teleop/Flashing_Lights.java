package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.SubSystem.RGBLights;

@TeleOp(name="FlashingLights")
public class Flashing_Lights extends OpMode {

    @Override public void init(){
        RGBLights lights = new RGBLights(hardwareMap);

}
@Override public void loop(){

}
}
