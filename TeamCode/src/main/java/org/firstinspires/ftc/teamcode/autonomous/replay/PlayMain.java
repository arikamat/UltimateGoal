package org.firstinspires.ftc.teamcode.autonomous.replay;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.utils.MecanumDrive;
import org.firstinspires.ftc.teamcode.utils.odometry.GlobalPosition;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.util.Scanner;


public class PlayMain extends LinearOpMode {
    Scanner input;
    MecanumDrive mecDrive;
    long start;
    GlobalPosition gps;
    double timeDiff, timeStamp;
    boolean onTime = true;
    public void initi() throws FileNotFoundException {
        gps = new GlobalPosition(hardwareMap,75);
        mecDrive = new MecanumDrive(hardwareMap,false);
        input = new Scanner(new File(Record.autoFile));
        input.useDelimiter(",|\\n");
        start = System.currentTimeMillis();
    }
    @Override
    public void runOpMode() {
        try {
            initi();
        }catch(IOException e){
            e.printStackTrace();
        }
        while(opModeIsActive()){
            if(input!=null && input.hasNextDouble()){
                if(onTime){
                    timeStamp = input.nextDouble();
                }
                timeDiff = timeStamp-(System.currentTimeMillis()-start);
                if(timeDiff<=0){
                    gps.goToPosition(input.nextDouble(),input.nextDouble(),0.5,input.nextDouble(),1);
                    onTime=true;
                }
                else{
                    onTime=false;
                }
            }
            else{
                this.end();
                if (input != null) {
                    input.close();
                    input = null;
                }
            }
        }
    }
    public void end() {
        mecDrive.setPower(0,0,0,0);
        if (input != null) {
            input.close();
        }
    }

}


