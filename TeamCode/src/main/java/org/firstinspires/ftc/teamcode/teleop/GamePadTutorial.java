package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "GamePadTutorial", group="TeleOp")
public class GamePadTutorial extends OpMode {

    @Override
    public void init() {

    }

    @Override
    public void loop() {
        if(gamepad1.a){
            telemetry.addLine("A is Pressed on Gamepad1");
        }
        if(gamepad1.b){
            telemetry.addLine("B is Pressed on Gamepad1");
        }
        if(gamepad1.dpad_up){
            telemetry.addLine("dpad_up is Pressed on Gamepad1");
        }
        if(Math.abs(gamepad1.left_stick_x)>0.15){
            telemetry.addLine("left joystick has moved significantly in x direction");
        }
        if(gamepad2.a){
            telemetry.addLine("A is pressed on Gamepad2");
        }
        telemetry.update();

    }
}


