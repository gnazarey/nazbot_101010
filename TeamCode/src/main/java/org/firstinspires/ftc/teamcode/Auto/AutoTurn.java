package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.SubSystem.Odometer;
import org.firstinspires.ftc.teamcode.Utils.Constants;
import org.firstinspires.ftc.teamcode.SubSystem.DriveTrain;
import org.firstinspires.ftc.teamcode.Utils.Logger;
import org.firstinspires.ftc.teamcode.Utils.Types;

import java.util.Locale;

@Autonomous(name = "TurnTest")
public class AutoTurn extends OpMode {
    private DriveTrain driveTrain;
    private int current_step = 0;
    private Types.WayPoint[] wayPoints;
    private Logger logger;
    private boolean settime = false;
    private double startTime;
    private Odometer odometer;

    public void init() {
        // Setup Logging
        this.logger = new Logger("autoturn.log",true);
        this.logger.writeLog("Auto Turning Starting");
        // Setup odometer
        this.odometer = new Odometer(hardwareMap,this.logger);
        this.odometer.reset();
        this.odometer.update();
        // Setup Logging
        this.logger = new Logger("autoturn.log",true);
        this.logger.writeLog("Auto Turning Starting");
        // Setup DriveTrain
        driveTrain = new DriveTrain(hardwareMap);
        // Setup Waypoints
        this.wayPoints = new Types.WayPoint[10];
        this.wayPoints[0] = new Types.WayPoint();
        this.wayPoints[0].facing = Constants.DriveBase.Headings.NORTH;
        this.wayPoints[1] = new Types.WayPoint();
        this.wayPoints[1].facing = Constants.DriveBase.Headings.WEST;
        this.wayPoints[2] = new Types.WayPoint();
        this.wayPoints[2].facing = Constants.DriveBase.Headings.SOUTH;
        this.wayPoints[3] = new Types.WayPoint();
        this.wayPoints[3].facing = Constants.DriveBase.Headings.EAST;

        this.current_step = 1;
    }

    public void loop(){
        // Loop Update
        this.odometer.update();
        this.driveTrain.loop();

        if (!this.settime){
            startTime = getRuntime();
            this.settime = true;
        }
        else {
            driveTrain.setDirection(this.wayPoints[this.current_step].facing);
            if (getRuntime() - startTime >= 1.0)
            {
                this.current_step++;
                this.settime = false;
            }
        }
        if (this.current_step > 3){
            this.current_step = 0;
        }

    }
}