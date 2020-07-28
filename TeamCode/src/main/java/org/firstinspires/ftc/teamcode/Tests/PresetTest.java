package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.teamcode.utils.PIDController;

@TeleOp(name="Preset Test", group = "Tests")
public class PresetTest extends OpMode {
    PIDController pid;
    DcMotor motor;
    PIDCoefficients coeff;
    @Override
    public void init(){
        motor = hardwareMap.get(DcMotor.class, "arm");
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        coeff = new PIDCoefficients(0,0,0);
        pid = new PIDController(motor,coeff);
    }
    @Override
    public void loop(){
        if(gamepad1.dpad_up) {
            pid.moveToPos(308, telemetry);
        }
    }
}
