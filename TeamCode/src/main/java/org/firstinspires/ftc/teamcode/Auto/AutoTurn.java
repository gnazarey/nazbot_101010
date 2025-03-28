package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.SubSystem.Odometer;
import org.firstinspires.ftc.teamcode.Utils.Constants;
import org.firstinspires.ftc.teamcode.SubSystem.DriveTrain;
import org.firstinspires.ftc.teamcode.Utils.Logger;
import org.firstinspires.ftc.teamcode.Utils.Types;

import java.util.Locale;

@Config
@Autonomous(name = "TurnTest")
public class AutoTurn extends OpMode {
    private DriveTrain driveTrain;
    private int current_step = 0;
    private Types.WayPoint[] wayPoints;
    private Logger logger;
    private boolean setTime = false;
    private boolean newTurn = true;
    private double startTime;
    private Odometer odometer;
    private Telemetry telemetry;
    public static double pause_time;
    private int counter = 0;

    public void init_loop(){
        this.counter++;
        if ((this.counter % 1000) == 0) {
            this.odometer.reset();
            this.odometer.update();
        }
        if (this.odometer.getY() <= 0.1 && this.odometer.getX() <= 0. && this.odometer.getHeading() <= 0.1){
            this.telemetry.addData("Robot Good ",this.odometer.getHeading());
        } else {
            this.telemetry.addData("X: ",this.odometer.getX());
            this.telemetry.addData("Y: ",this.odometer.getY());
            this.telemetry.addData("Heading: ",this.odometer.getHeading());
        }
        this.telemetry.update();
    }
    public void init() {
        // Setup Logging
        this.logger = new Logger("autoturn.log",true);
        this.logger.writeLog("Auto Turning Starting");
        // Setup odometer
        this.odometer = new Odometer(hardwareMap,this.logger);
        this.odometer.reset();
        this.odometer.update();
        // Setup Telemetry
        this.telemetry = FtcDashboard.getInstance().getTelemetry();
        // Setup DriveTrain
        driveTrain = new DriveTrain(hardwareMap,this.logger,this.odometer,this.telemetry);
        driveTrain.setMaxXSpeed(Constants.DriveBase.MAX_SPEED);
        driveTrain.setMaxYSpeed(Constants.DriveBase.MAX_SPEED);
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
        AutoTurn.pause_time = 2.0;
    }

    public void loop(){
        // Telemetry
        this.telemetry.addData("X: ",this.odometer.getX());
        this.telemetry.addData("Y: ",this.odometer.getY());
        // Loop Update
        this.odometer.update();
        this.driveTrain.loop();

        if (!this.setTime){
            this.startTime = getRuntime();
            this.setTime = true;
        }
        else {
            driveTrain.setDirection(this.wayPoints[this.current_step].facing);
            if (this.newTurn) {
                this.logger.writeLog(String.format(Locale.ENGLISH, "Turning to, %f", this.wayPoints[this.current_step].facing));
                this.telemetry.addData("Turning to:",this.wayPoints[this.current_step].facing);
                this.newTurn = false;
            }
            if (getRuntime() - this.startTime >= AutoTurn.pause_time)
            {
                this.current_step++;
                this.setTime = false;
                this.newTurn = true;
            }
        }
        if (this.current_step > 3){
            this.current_step = 0;
        }
    }
}