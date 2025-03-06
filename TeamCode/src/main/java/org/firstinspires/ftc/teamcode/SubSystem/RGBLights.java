package org.firstinspires.ftc.teamcode.SubSystem;

import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.Utils.Constants;
import org.firstinspires.ftc.teamcode.Utils.Logger;

/////////////////////////////////////////////////
// This is the RGB subsystem it controls the RGB
// lights.
//
// Written By: George Nazarey
// Date: 3/4/2025
// Email: george@nazarey.ca
//

public class RGBLights {
    private final SimpleServo lRGB;
    private final SimpleServo rRGB;
    private double lRGBColor;
    private double rRGBColor;
    private Logger logger;

    public RGBLights(HardwareMap hwMap){
        this.lRGB = new SimpleServo(hwMap,"lrgb",0.0,1.0);
        this.rRGB = new SimpleServo(hwMap,"rrgb",0.0,1.0);
        if (Constants.RGB_Light.LOGGING) {
            this.logger = new Logger(Constants.RGB_Light.LOGFILE);
        }
        this.init();
    }

    private void init(){
        this.rRGB.setPosition(Constants.RGB_Light.WHITE);
        this.rRGBColor = Constants.RGB_Light.WHITE;
        this.lRGB.setPosition(Constants.RGB_Light.WHITE);
        this.lRGBColor = Constants.RGB_Light.WHITE;
        if (Constants.RGB_Light.LOGGING){
            this.logger.writeLog("RGB Initialized\n");
        }
        if (!this.logger.getLogging()){
            this.rRGB.setPosition(Constants.RGB_Light.RED);
        }
    }

    public void setlRGB(double color){
        this.lRGB.setPosition(color);
        this.lRGBColor = color;
    }

    public void setrRGB(double color){
        this.rRGB.setPosition(color);
        this.rRGBColor = color;
    }

    public void setAllRGB(double color){
        this.lRGB.setPosition(color);
        this.lRGBColor = color;
        this.rRGB.setPosition(color);
        this.rRGBColor = color;
    }

    public double getlRGBColor(){
        return this.lRGBColor;
    }

    public double getrRGBColor(){
        return this.rRGBColor;
    }
}
