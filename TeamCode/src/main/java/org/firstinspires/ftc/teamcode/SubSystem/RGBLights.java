package org.firstinspires.ftc.teamcode.SubSystem;


import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.Utils.Constants;

public class RGBLights {
    private final SimpleServo lRGB;
    private final SimpleServo rRGB;

    public RGBLights(HardwareMap hwMap){
        this.lRGB = new SimpleServo(hwMap,"lrgb",0.0,1.0);
        this.rRGB = new SimpleServo(hwMap,"rrgb",0.0,1.0);
        this.init();
    }

    private void init(){
        this.rRGB.setPosition(Constants.RGB_Light.WHITE);
        this.lRGB.setPosition(Constants.RGB_Light.WHITE);
    }

    public void setlRGB(double color){
        this.lRGB.setPosition(color);
    }

    public void setrRGB(double color){
        this.rRGB.setPosition(color);
    }
}
