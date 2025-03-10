package org.firstinspires.ftc.teamcode.Test;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Utils.Logger;

import java.util.Locale;

// This is a command test class
public class Command_Base extends CommandBase {
    // Setup Logging
    private Logger logger;

    public void init(){
        this.logger = new Logger("command_base_operations.log");
        this.logger.writeLog(String.format(Locale.ENGLISH,"%s COMMAND - Initialized",this.logger.getTimeStamp()));
    }
    @Override public boolean isFinished(){
        return true;
    }
}
