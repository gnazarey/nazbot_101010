package org.firstinspires.ftc.teamcode.Test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.SubSystem.Odometer;

@TeleOp(name="Test_Odometer")
public class odometer extends OpMode {
    private Odometer odometer;
    private Telemetry telemetry;

    public void init(){
        this.telemetry = FtcDashboard.getInstance().getTelemetry();
        this.odometer = new Odometer(hardwareMap);
    }
    public void loop(){
        this.telemetry.addData("X: ",this.odometer.getX());
        this.telemetry.addData("Y: ",this.odometer.getY());
        this.telemetry.addData("Heading: ",this.odometer.getHeading());
        this.telemetry.update();
        this.odometer.update();
    }
}
