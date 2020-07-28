package org.firstinspires.ftc.teamcode.utils.odometry;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.utils.CONFIG;
import org.firstinspires.ftc.teamcode.utils.Encoders;
import org.firstinspires.ftc.teamcode.utils.IMU;
import org.firstinspires.ftc.teamcode.utils.MecanumDrive;

import java.io.File;

@Autonomous(name = "Odometry System Calibration" , group = "Util")
public class Calibration extends LinearOpMode {
    MecanumDrive drive;
    Encoders encoders;
    IMU imu;

    final double PIVOT_SPEED = 0.5;

    static final double TICKS_PER_REV = CONFIG.COUNTS_PER_REVOLUTION;
    static final double WHEEL_DIAMETER = 1.5;
    static final double GEAR_RATIO= 1;
    final double COUNTS_PER_INCH = WHEEL_DIAMETER*Math.PI*GEAR_RATIO/TICKS_PER_REV;

    ElapsedTime timer = new ElapsedTime();

    double horizontalTickOffset = 0;

    //Text files to write the values to. The files are stored in the robot controller under Internal Storage\FIRST\settings
    File wheelBaseSeparationFile = AppUtil.getInstance().getSettingsFile("wheelBaseSeparation.txt");
    File horizontalTickOffsetFile = AppUtil.getInstance().getSettingsFile("horizontalTickOffset.txt");

    @Override
    public void runOpMode() throws InterruptedException {
        imu = new IMU(hardwareMap);
        drive = new MecanumDrive(hardwareMap, false);

        telemetry.addData("IMU Status", imu.initializeIMU());
        telemetry.addData("Odometry System Calibration Status", "Init Complete");
        telemetry.update();

        waitForStart();

        while (imu.getZAngle() < 90 && opModeIsActive()) {
            drive.setPower(PIVOT_SPEED, -PIVOT_SPEED, PIVOT_SPEED, -PIVOT_SPEED);
            if (imu.getZAngle() < 60) {
                drive.setPower(PIVOT_SPEED, -PIVOT_SPEED, PIVOT_SPEED, -PIVOT_SPEED);
            } else {
                drive.setPower(PIVOT_SPEED / 2, -PIVOT_SPEED / 2, PIVOT_SPEED / 2, -PIVOT_SPEED / 2);
            }
        }
        telemetry.addData("IMU Angle", imu.getZAngle());
        telemetry.update();
        //Stop the robot
        drive.setPower(0, 0, 0, 0);
        timer.reset();
        while (timer.milliseconds() < 1000 && opModeIsActive()) {
            telemetry.addData("IMU Angle", imu.getZAngle());
            telemetry.update();
        }

        //Record IMU and encoder values to calculate the constants for the global position algorithm
        double angle = imu.getZAngle();

        /*
        Encoder Difference is calculated by the formula (leftEncoder - rightEncoder)
        Since the left encoder is also mapped to a drive motor, the encoder value needs to be reversed with the negative sign in front
        THIS MAY NEED TO BE CHANGED FOR EACH ROBOT
       */
        double leftAbs = Math.abs(encoders.getLeft());
        double rightAbs = Math.abs(encoders.getRight());
        double encoderDifference = Math.abs(leftAbs - rightAbs);

        double verticalEncoderTickOffsetPerDegree = encoderDifference / angle;

        double wheelBaseSeparation = (2 * 90 * verticalEncoderTickOffsetPerDegree) / (Math.PI * COUNTS_PER_INCH);

        horizontalTickOffset = encoders.getHorizontal() / Math.toRadians(imu.getZAngle());

        //Write the constants to text files
        ReadWriteFile.writeFile(wheelBaseSeparationFile, String.valueOf(wheelBaseSeparation));
        ReadWriteFile.writeFile(horizontalTickOffsetFile, String.valueOf(horizontalTickOffset));

        while (opModeIsActive()) {
            telemetry.addData("Odometry System Calibration Status", "Calibration Complete");
            //Display calculated constants
            telemetry.addData("Wheel Base Separation", wheelBaseSeparation);
            telemetry.addData("Horizontal Encoder Offset", horizontalTickOffset);

            //Display raw values
            telemetry.addData("IMU Angle", imu.getZAngle());
            telemetry.addData("Vertical Left Position", -encoders.getLeft());
            telemetry.addData("Vertical Right Position", encoders.getRight());
            telemetry.addData("Horizontal Position", encoders.getHorizontal());
            telemetry.addData("Vertical Encoder Offset", verticalEncoderTickOffsetPerDegree);

            //Update values
            telemetry.update();
        }
    }
}

