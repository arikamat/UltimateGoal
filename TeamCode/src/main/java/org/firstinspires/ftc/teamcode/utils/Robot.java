package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utils.odometry.GlobalPosition;

public class Robot {
    PIDController PID;
    MecanumDrive drive;
    IMU imu;
    Encoders encoders;
    GlobalPosition gps;
    Telemetry telemetry;
    public Robot(Telemetry telemetry, HardwareMap hmap){
        drive = new MecanumDrive(hmap,false);
        imu=new IMU(hmap);
        this.telemetry=telemetry;
        encoders=new Encoders(hmap);
        gps =new GlobalPosition(hmap,75);
        Thread thread=new Thread(gps);
        thread.start();
    }
}
