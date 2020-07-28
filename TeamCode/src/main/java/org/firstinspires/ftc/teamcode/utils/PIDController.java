package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDController extends LinearOpMode {
    ElapsedTime PIDTimer = new ElapsedTime();
    Encoders encode;
    MecanumDrive drive;
    double ticks = CONFIG.COUNTS_PER_REVOLUTION;
    double gearRatio = 1;
    double radius = 3;
    double ticks2Inch = (1 / ticks) * gearRatio * (2 * Math.PI * radius);

    @Override
    public void runOpMode() {
        encode = new Encoders(hardwareMap);
        drive = new MecanumDrive(hardwareMap, false);
        waitForStart();

    }

    public void moveMotor(double targetPosition) {
        if (gamepad1.b) {

        } else if (gamepad1.x) {

        }
    }
}
