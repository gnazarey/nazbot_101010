package org.firstinspires.ftc.teamcode.SubSystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class RGBLightsSubSystem extends SubsystemBase {
    private final SimpleServo lRGB;
    private double lRGBColor;
    private double rRGBColor;

    public RGBLightsSubSystem(HardwareMap hwMap) {
        this.lRGB = new SimpleServo(hwMap,"lrgb",0.0,1.0);
    }
    public void stop(){

    }
}
