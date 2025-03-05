package org.firstinspires.ftc.teamcode.Utils;

import android.os.Environment;

import java.io.FileWriter;
import java.io.IOException;

/////////////////////////////////////////////////
// This is the logging module.
//
// Written By: George Nazarey
// Date: 3/5/2025
// Email: george@nazarey.ca
//
public class Logger {

    private final String fileName;
    private FileWriter logWriter;
    private Boolean logging = true;

    public Logger(String filename){

        this.fileName = String.format("%s/FIRST/data/'%s", Environment.getExternalStorageDirectory().getAbsoluteFile(),filename);
        this.init();
    }

    private void init(){
        try{
            this.logWriter = new FileWriter(this.fileName,false);
        }
        catch (IOException e) {
            this.logging = false;
        }
    }

    public int writeLog(String logLine)
    {
        int outputCount = 0;

        if (this.logging) {
            try {
                this.logWriter.write(logLine);
                outputCount = logLine.length();
            }
            catch (IOException e) {
                outputCount = -1;
                this.logging = false;
            }
        }
        return outputCount;
    }

    public void stopLog() {

        if (this.logging) {
            try {
                this.logWriter.flush();
                this.logWriter.close();
            }
            catch (IOException e) {
                this.logging = false;
            }
        }
    }
}
