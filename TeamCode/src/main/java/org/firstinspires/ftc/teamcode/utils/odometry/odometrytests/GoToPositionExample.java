package org.firstinspires.ftc.teamcode.utils.odometry.odometrytests;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.utils.CONFIG;
import org.firstinspires.ftc.teamcode.utils.Encoders;
import org.firstinspires.ftc.teamcode.utils.IMU;
import org.firstinspires.ftc.teamcode.utils.MecanumDrive;
import org.firstinspires.ftc.teamcode.utils.odometry.GlobalPosition;

@Autonomous(name = "GoToPos", group = "Tests")
public class GoToPositionExample extends LinearOpMode {
    GlobalPosition gps;
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
        gps = new GlobalPosition(hardwareMap,75);
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
    public void goToPosition(double targetX, double targetY, double power, double desiredOrientation, double marginOfError){
        //DesiredOrientation is the angle you want the robot to be at while going to (targetX,targetY).
        //For example if you were strafing, the robot tends to turn a bit, this would make sure you are going straight
        double distanceToX = targetX - gps.getXCoordinate();
        double distanceToY = targetY - gps.getYCoordinate();
        double distance = Math.hypot(distanceToX,distanceToY);
        while(opModeIsActive() && distance>marginOfError) {
            distance = Math.hypot(distanceToX,distanceToY);
            distanceToX = targetX - gps.getXCoordinate();
            distanceToY = targetY - gps.getYCoordinate();
            double travelAngle = Math.toDegrees(Math.atan2(distanceToX, distanceToY)); //For example if i am at 2,2 and want to go to 10,10. I would want to travel at a 45 degree angle
            //0 degrees is up rather than right in a unit circle
            double xComp = calculateXComponent(travelAngle, power);
            double yComp = calculateYComponent(travelAngle, power);

            double pivotCorrection = desiredOrientation - gps.getOrientationDegrees(); //Pivots so that it maintains the desiredOrientation while travelling to position
            drive.XYCorrection(xComp,yComp,pivotCorrection);
        }
    }
    public double calculateXComponent(double angle, double power){
        return Math.sin(Math.toRadians(angle))*power; //This is sin because you want 0 degrees to go up rather than east
    }
    public double calculateYComponent(double angle, double power){
        return Math.cos(Math.toRadians(angle))*power; //This is cos because you want 0 degrees to go up rather than east
    }

}
