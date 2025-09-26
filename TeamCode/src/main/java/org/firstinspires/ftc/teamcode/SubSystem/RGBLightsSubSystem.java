package org.firstinspires.ftc.teamcode.SubSystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Utils.Constants;

public class RGBLightsSubSystem extends SubsystemBase {
    private final SimpleServo RGBleft;
    private final SimpleServo RGBright;
    private double rgbLeftColor = Constants.RGB_Light.OFF;
    private double rgbRightColor = Constants.RGB_Light.OFF;

    public RGBLightsSubSystem(HardwareMap hwMap) {
        this.RGBleft = new SimpleServo(hwMap,"lrgb",0.0,1.0);
        this.RGBright = new SimpleServo(hwMap,"rrgb",0.0,1.0);
    }
    // Get Section
    public double getColor(double color,boolean left, boolean right) {
        if (left){
            return this.rgbLeftColor;
        }
        if (right){
            return this.rgbRightColor;
        }
        return Constants.RGB_Light.ERROR;
    }
    // Set Section
    // This sets the color to the given color
    public void setColor(double color,boolean left, boolean right) {
        if (left){
            this.RGBleft.setPosition(color);
            this.rgbLeftColor = color;
        }
        if (right){
            this.RGBright.setPosition(color);
            this.rgbRightColor = color;
        }
    }
    // Method Section
    // This turns the light on
    public void turnOn(){
        this.setColor(Constants.RGB_Light.ON,true,true);
    }
    // This turns the light off
    public void turnOff(){
        this.setColor(Constants.RGB_Light.OFF,true,true);
    }
    // This turns the light off
    public void stop(){
        this.turnOff();
    }
}
