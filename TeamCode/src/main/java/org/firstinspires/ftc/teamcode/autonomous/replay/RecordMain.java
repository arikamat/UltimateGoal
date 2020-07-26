package org.firstinspires.ftc.teamcode.autonomous.replay;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.utils.CONFIG;
import org.firstinspires.ftc.teamcode.utils.MecanumDrive;
import org.firstinspires.ftc.teamcode.utils.odometry.GlobalPosition;

import java.io.FileWriter;
import java.io.IOException;

public class RecordMain extends LinearOpMode {
    private MecanumDrive mecDrive;
    GlobalPosition gps;
    String autoFile = "replayData.csv";
    FileWriter writer;
    Record recorder;
    DcMotor motorLF,motorLB, motorRF,motorRB,   odoLeft,odoRight,odoHorizontal;
    BNO055IMU imu;
    double COUNTS_PER_INCH = CONFIG.COUNTS_PER_INCH;
    long startTime;

    public void initi(){
        mecDrive = new MecanumDrive(hardwareMap,false);
        try {
            recorder = new Record(hardwareMap); // initializes record mechanism
        }
        catch(IOException e){
            telemetry.addLine("IOEXCEPTION");
            telemetry.update();
        }
    }

    @Override
    public void runOpMode(){
        initi();
        while(opModeIsActive()) {
            mecDrive.teleopTank(gamepad1,0.5); //does the math and calculates how to move from the joystick values
            try {
                recorder.record(); //records current global position and records into a csv file separated by a comma
            }
            catch (IOException e) {
                e.printStackTrace();
            }
        }
        try{
            if(recorder != null){
                recorder.end();
            }
        }
        catch (IOException e){
            e.printStackTrace();
        }
    }
}
