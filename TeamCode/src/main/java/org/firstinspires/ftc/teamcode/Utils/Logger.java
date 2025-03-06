package org.firstinspires.ftc.teamcode.Utils;

import android.os.Environment;
import java.io.FileNotFoundException;
import java.io.PrintStream;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Locale;

/////////////////////////////////////////////////
// This is the logging module.
//
// Written By: George Nazarey
// Date: 3/5/2025
// Email: george@nazarey.ca
//
public class Logger {

    //private final String fileName = "/sdcard/FIRST/data/test.log";
    private final String fileName;
    private PrintStream logWriter;
    private Boolean logging = true;
    private final SimpleDateFormat dateFormat;

    public Logger(String filename){

        this.fileName = String.format("%s/FIRST/data/%s",Environment.getExternalStorageDirectory().getPath(), filename);
        this.dateFormat = new SimpleDateFormat("yyyy-MM-dd@HH-mm-ss", Locale.US);
        this.init();
    }

    private void init(){
        try{
            this.logWriter = new PrintStream(this.fileName);
        }
        catch (FileNotFoundException e) {
            this.logging = false;
        }
    }

    public boolean getLogging(){
        return this.logging;
    }

    public String getTimeStamp(){
        return this.dateFormat.format(new Date());
    }

    public void writeLog(String logLine)
    {
        if (this.logging) {
            this.logWriter.println(logLine);
            }
    }

    public void stopLog() {
        if (this.logging) {
            this.logWriter.flush();
            this.logWriter.close();
        }
    }
}
