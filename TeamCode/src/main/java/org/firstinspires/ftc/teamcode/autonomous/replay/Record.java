package org.firstinspires.ftc.teamcode.autonomous.replay;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utils.CONFIG;
import org.firstinspires.ftc.teamcode.utils.MecanumDrive;
import org.firstinspires.ftc.teamcode.utils.odometry.GlobalPosition;

import java.io.FileWriter;
import java.io.IOException;

public class Record {
    long startTime;
    FileWriter writer;
    MecanumDrive drive;
    public static String autoFile = "replayData.csv";
    GlobalPosition gps;
    public Record(HardwareMap hmap) throws IOException {
        startTime = System.currentTimeMillis();
        writer=new FileWriter(autoFile);
        double COUNTS_PER_INCH = CONFIG.COUNTS_PER_INCH;
        gps = new GlobalPosition(hmap, 75);
        Thread positionThread = new Thread(gps);
        positionThread.start();
        drive = new MecanumDrive(hmap,false);
    }
    public void record() throws IOException{
        if(writer!=null) {
            writer.append("" + (System.currentTimeMillis() - startTime));
            writer.append("," + gps.getX());
            writer.append("," + gps.getY());
            writer.append("," + gps.getOrientationDegrees());
            writer.append("\n");

            //BELOW IS WHAT YOU SHOULD USE IN THE CASE ODOMETRY DOESN'T WORK OUT :`(
                /*writer.append("" + (System.currentTimeMillis() - startTime));
                writer.append("," + drive.getFLPos());
                writer.append("," + drive.getFRPos());
                writer.append("," + drive.getBLPos());
                writer.append("," + drive.getBRPos());
                writer.append("\n");*/
           //END WORST CASE SCENARIO
        }
    }
    public void end() throws IOException{
        if(writer !=null) {
            writer.flush();
            writer.close();
        }
    }
}