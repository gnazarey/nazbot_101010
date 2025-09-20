package org.firstinspires.ftc.teamcode.TeleopCommands;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Commands.RGBLightsOnCommand;
import org.firstinspires.ftc.teamcode.SubSystem.RGBLightsSubSystem;

@TeleOp(name="RGBLightsOn_cmd", group = "CommandMode")
public class RGBLightsOn extends CommandOpMode {

    private RGBLightsSubSystem rgbLights;
    private RGBLightsOnCommand lightCommand;

    @Override
    public void initialize(){
        rgbLights = new RGBLightsSubSystem(hardwareMap);

        lightCommand = new RGBLightsOnCommand(rgbLights);

        register(rgbLights);

        rgbLights.setDefaultCommand(lightCommand);
    }
}
