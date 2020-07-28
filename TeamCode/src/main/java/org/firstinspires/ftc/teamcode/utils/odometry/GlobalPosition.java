package org.firstinspires.ftc.teamcode.utils.odometry;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.utils.CONFIG;
import org.firstinspires.ftc.teamcode.utils.Encoders;

import java.io.File;

/**
 * Created by Sarthak on 6/1/2019.
 */
public class GlobalPosition implements Runnable{
    //Odometry wheels
    Encoders encoder;

    //Thread run condition
    private boolean isRunning = true;

    //Position variables used for storage and calculations
    static final double TICKS_PER_REV = CONFIG.COUNTS_PER_REVOLUTION;
    static final double WHEEL_DIAMETER = 1.5;
    static final double GEAR_RATIO= 1;
    final double COUNTS_PER_INCH = WHEEL_DIAMETER*Math.PI*GEAR_RATIO/TICKS_PER_REV;

    double rightEncoderPos = 0, leftEncoderPos = 0, horEncoderPos = 0;
    double changeInRobotOrientation = 0;
    private double globalX = 0, globalY = 0, globalRadians = 0;
    private double oldRightEncoderPos = 0, oldLeftEncoderPos = 0, oldHorEncoderPos = 0;

    //Algorithm constants
    private double encoderWheelDistance;
    private double horizontalOffset;

    //Sleep time interval (milliseconds) for the position update thread
    private int sleepTime;

    //Files to access the algorithm constants
    private File wheelBaseSeparationFile = AppUtil.getInstance().getSettingsFile("wheelBaseSeparation.txt");
    private File horizontalTickOffsetFile = AppUtil.getInstance().getSettingsFile("horizontalTickOffset.txt");

    private int verticalLeftEncoderPositionMultiplier = 1;
    private int verticalRightEncoderPositionMultiplier = 1;
    private int normalEncoderPositionMultiplier = 1;
    public GlobalPosition(HardwareMap hmap, int threadSleepDelay){
        this.encoder = new Encoders(hmap);
        this.sleepTime = threadSleepDelay;

        encoderWheelDistance = Double.parseDouble(ReadWriteFile.readFile(wheelBaseSeparationFile).trim()) * this.COUNTS_PER_INCH;
        this.horizontalOffset = Double.parseDouble(ReadWriteFile.readFile(horizontalTickOffsetFile).trim());
    }

    public void globalCoordinatePositionUpdate() {
        //Get Current Positions
        leftEncoderPos = (encoder.getLeft()* verticalLeftEncoderPositionMultiplier);
        rightEncoderPos = (encoder.getRight() * verticalRightEncoderPositionMultiplier);

        double leftChange = leftEncoderPos - oldLeftEncoderPos;
        double rightChange = rightEncoderPos - oldRightEncoderPos;

        //Calculate Angle
        changeInRobotOrientation = (leftChange - rightChange) / (encoderWheelDistance);
        globalRadians += changeInRobotOrientation;

        //Get the components of the motion
        horEncoderPos = (encoder.getHorizontal()*normalEncoderPositionMultiplier);
        double rawHorizontalChange = horEncoderPos - oldHorEncoderPos;
        double horizontalChange = rawHorizontalChange - (changeInRobotOrientation* horizontalOffset);

        double sides = ((rightChange + leftChange) / 2);
        double n = horizontalChange;

        //Calculate and update the position values
        globalX = globalX + (sides*Math.sin(globalRadians) + n*Math.cos(globalRadians));
        globalY = globalY + (sides*Math.cos(globalRadians) - n*Math.sin(globalRadians));

        oldLeftEncoderPos = leftEncoderPos;
        oldRightEncoderPos = rightEncoderPos;
        oldHorEncoderPos = horEncoderPos;
    }

    public double getX() { return globalX; }
    public double getXCoordinate(){
        return globalX / COUNTS_PER_INCH;
    }
    public double getY() { return globalY; }
    public double getYCoordinate(){
        return globalY / COUNTS_PER_INCH;
    }
    public double getOrientationDegrees() { return Math.toDegrees(globalRadians) % 360; }
    public double getOrientationRadians(){ return globalRadians %(Math.PI*2); }

    public void stop() { isRunning = false; }

    public void reverseLeft() {
        verticalLeftEncoderPositionMultiplier = -verticalLeftEncoderPositionMultiplier;
    }

    public void reverseRight() {
        verticalRightEncoderPositionMultiplier = -verticalRightEncoderPositionMultiplier;
    }

    public void reverseHorizontal() {
        normalEncoderPositionMultiplier = -normalEncoderPositionMultiplier;
    }
    public void goTo(double x, double y, double angle){
        int a=1;
        //add stuff here later
    }
    @Override
    public void run() {
        while(isRunning) {
            globalCoordinatePositionUpdate();
            try {
                Thread.sleep(sleepTime);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }
}