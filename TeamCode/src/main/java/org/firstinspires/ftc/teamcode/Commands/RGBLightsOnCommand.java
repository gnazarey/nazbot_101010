package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.SubSystem.RGBLightsSubSystem;

public class RGBLightsOnCommand extends CommandBase {
    private final RGBLightsSubSystem RGBLights;
    private boolean finished;
    private double RGBColor;


    public RGBLightsOnCommand(RGBLightsSubSystem RGBLightsSubSystem){
        this.RGBLights = RGBLightsSubSystem;
        this.finished = false;
        addRequirements(RGBLights);
    }

    @Override
    public void initialize(){
        this.finished = false;
    }

    @Override
    public void execute(){
        this.RGBLights.turnOn();
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

