package org.firstinspires.ftc.teamcode.utils.odometry.odometrytests;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.utils.CONFIG;
import org.firstinspires.ftc.teamcode.utils.odometry.GlobalPosition;

@TeleOp(name = "Passive GPS Test", group = "Tests")
public class PassiveCoordinateLocater extends LinearOpMode {
    private GlobalPosition gps;
    final double COUNTSPERINCH =307.69957;
    BNO055IMU imu;
    DcMotor FL,FR,BL,BR, odoL, odoR, odoH;
    public void initialize(){
        FL = hardwareMap.get(DcMotor.class, CONFIG.FRONTLEFT);
        FR = hardwareMap.get(DcMotor.class, CONFIG.FRONTRIGHT);
        BL = hardwareMap.get(DcMotor.class, CONFIG.BACKLEFT);
        BR = hardwareMap.get(DcMotor.class, CONFIG.BACKRIGHT);
        odoL = hardwareMap.get(DcMotor.class, CONFIG.LEFTVERTICAL);
        odoH =hardwareMap.get(DcMotor.class, CONFIG.HORIZONTAL);
        odoR =hardwareMap.get(DcMotor.class, CONFIG.RIGHTVERTICAL);
        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        odoH.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        odoL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        odoR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //---
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //---
        odoL.setDirection(DcMotorSimple.Direction.REVERSE);
        odoH.setDirection(DcMotorSimple.Direction.REVERSE);
        odoR.setDirection(DcMotorSimple.Direction.REVERSE);

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(parameters);
        telemetry.addData("IMU Setup", "Initialization is Complete");
        telemetry.update();

    }
    @Override
    public void runOpMode(){
        initialize();
        waitForStart();
        GlobalPosition gps = new GlobalPosition(hardwareMap,75);
        Thread pos = new Thread(gps);
        pos.start();
        while(opModeIsActive()){
            telemetry.addData("X: ", gps.getX());
            telemetry.addData("Y: ", gps.getY());
            telemetry.addData("Angle (Degrees): ", gps.getOrientationDegrees());
            telemetry.addData("Angle (Radians): ", gps.getOrientationRadians());
            telemetry.update();
        }
        gps.stop();
    }

}
