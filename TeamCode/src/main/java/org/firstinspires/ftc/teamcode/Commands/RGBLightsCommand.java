package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.SubSystem.RGBLights;
import org.firstinspires.ftc.teamcode.SubSystem.RGBLightsSubSystem;

public class RGBLightsCommand extends CommandBase {
    private final RGBLightsSubSystem RGBLights;
    private final boolean finished = true;
    private double lRGBColor;
    private double rRGBColor;

    public RGBLightsCommand(RGBLightsSubSystem RGBLightsSubSystem){
        this.RGBLights = RGBLightsSubSystem;

        addRequirements(RGBLightsSubSystem);
    }

    @Override
    public void initialize(){

    }

    @Override
    public void execute(){

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
