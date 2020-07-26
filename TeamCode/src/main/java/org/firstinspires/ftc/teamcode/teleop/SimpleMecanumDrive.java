package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.utils.MecanumDrive;

public class SimpleMecanumDrive extends OpMode {
    MecanumDrive drive;
    @Override
    public void init(){
        drive = new MecanumDrive(hardwareMap,false);
    }
    public void loop(){
        drive.teleopTank(gamepad1);
    }
}
