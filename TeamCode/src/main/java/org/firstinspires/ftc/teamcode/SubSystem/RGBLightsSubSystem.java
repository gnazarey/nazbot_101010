package org.firstinspires.ftc.teamcode.SubSystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Utils.Constants;

public class RGBLightsSubSystem extends SubsystemBase {
    private final SimpleServo RGB;
    private double RGBColor = Constants.RGB_Light.OFF;

    public RGBLightsSubSystem(HardwareMap hwMap) {
        this.RGB = new SimpleServo(hwMap,"lrgb",0.0,1.0);
    }
    // This sets the color to the given color
    public void setColor(double color) {
        this.RGB.setPosition(color);
        this.RGBColor = color;
    }
    // This turns the light on
    public void turnOn(){
        this.RGB.setPosition(Constants.RGB_Light.ON);
        this.RGBColor = Constants.RGB_Light.ON;
    }
    // This turns the light off
    public void turnOff(){
        this.RGB.setPosition(Constants.RGB_Light.OFF);
        this.RGBColor = Constants.RGB_Light.OFF;
    }

    // This turns the light off
    public void stop(){
        this.turnOff();
    }
}
