package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utils.IMU;
@TeleOp(name="IMU ReadOuts",group="Tests")
public class IMUReadOut extends OpMode {
    IMU imu;
    @Override
    public void init(){
        imu = new IMU(hardwareMap);
        telemetry.addData("IMU Situation",imu.initializeIMU());
    }
    @Override
    public void loop(){
        telemetry.addData("Angle",imu.getZAngle());
        telemetry.update();
    }

}
