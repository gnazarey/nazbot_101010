package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.SubSystem.RGBLights;
import org.firstinspires.ftc.teamcode.SubSystem.RGBLightsSubSystem;

public class RGBLightsColorCommand extends CommandBase {
    private final RGBLightsSubSystem RGBLights;
    private boolean finished;
    private double RGBColor;


    public RGBLightsColorCommand(RGBLightsSubSystem RGBLightsSubSystem, double color){
        this.RGBLights = RGBLightsSubSystem;
        this.RGBColor = color;
        addRequirements(RGBLightsSubSystem);
    }

    @Override
    public void initialize(){
        this.finished = false;
    }

    @Override
    public void execute(){
        this.RGBLights.setColor(RGBColor);
    }

    @Override
    public boolean isFinished(){
        return this.finished;
    }

    @Override
    public void end(boolean interrupted){
        RGBLights.stop();
    }
}
