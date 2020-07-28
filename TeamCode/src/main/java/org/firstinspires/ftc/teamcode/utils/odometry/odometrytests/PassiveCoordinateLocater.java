package org.firstinspires.ftc.teamcode.utils.odometry.odometrytests;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.utils.CONFIG;
import org.firstinspires.ftc.teamcode.utils.Encoders;
import org.firstinspires.ftc.teamcode.utils.IMU;
import org.firstinspires.ftc.teamcode.utils.MecanumDrive;
import org.firstinspires.ftc.teamcode.utils.odometry.GlobalPosition;

@TeleOp(name = "Passive GPS Test", group = "Tests")
public class PassiveCoordinateLocater extends LinearOpMode {
    private GlobalPosition gps;
    MecanumDrive drive;
    IMU imu;
    Encoders encoders;
    static final double TICKS_PER_REV = CONFIG.COUNTS_PER_REVOLUTION;
    static final double WHEEL_DIAMETER = 1.5;
    static final double GEAR_RATIO= 1;
    final double COUNTS_PER_INCH = WHEEL_DIAMETER*Math.PI*GEAR_RATIO/TICKS_PER_REV;

    public void initialize(){
        imu = new IMU(hardwareMap);
        drive = new MecanumDrive(hardwareMap,false);
        encoders = new Encoders(hardwareMap);
        encoders.resetEncoders();
        telemetry.addData("IMU Setup", imu.initializeIMU());
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
            drive.teleopTank(gamepad1,0.5);
            telemetry.addData("X: ", gps.getXCoordinate());
            telemetry.addData("Y: ", gps.getYCoordinate());
            telemetry.addData("Angle (Degrees): ", gps.getOrientationDegrees());
            telemetry.addData("Angle (Radians): ", gps.getOrientationRadians());
            telemetry.update();
        }
        gps.stop();
    }

}
