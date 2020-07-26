package org.firstinspires.ftc.teamcode.utils.odometry;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.utils.CONFIG;

import java.io.File;

/**
 * Created by Sarthak on 6/1/2019.
 */
public class GlobalPosition implements Runnable{
    //Odometry wheels
    private DcMotor verticalEncoderLeft, verticalEncoderRight, horizontalEncoder;

    //Thread run condition
    private boolean isRunning = true;

    //Position variables used for storage and calculations
    double verticalRightEncoderWheelPosition = 0, verticalLeftEncoderWheelPosition = 0, normalEncoderWheelPosition = 0,  changeInRobotOrientation = 0;
    private double robotGlobalXCoordinatePosition = 0, robotGlobalYCoordinatePosition = 0, robotOrientationRadians = 0;
    private double previousVerticalRightEncoderWheelPosition = 0, previousVerticalLeftEncoderWheelPosition = 0, prevNormalEncoderWheelPosition = 0;

    //Algorithm constants
    private double robotEncoderWheelDistance;
    private double horizontalEncoderTickPerDegreeOffset;

    //Sleep time interval (milliseconds) for the position update thread
    private int sleepTime;

    //Files to access the algorithm constants
    private File wheelBaseSeparationFile = AppUtil.getInstance().getSettingsFile("wheelBaseSeparation.txt");
    private File horizontalTickOffsetFile = AppUtil.getInstance().getSettingsFile("horizontalTickOffset.txt");

    private int verticalLeftEncoderPositionMultiplier = 1;
    private int verticalRightEncoderPositionMultiplier = 1;
    private int normalEncoderPositionMultiplier = 1;
    public GlobalPosition(HardwareMap hmap, int threadSleepDelay){
        this.verticalEncoderLeft = hmap.get(DcMotor.class, CONFIG.LEFTVERTICAL);
        this.verticalEncoderRight = hmap.get(DcMotor.class, CONFIG.RIGHTVERTICAL);
        this.horizontalEncoder = hmap.get(DcMotor.class, CONFIG.HORIZONTAL);
        sleepTime = threadSleepDelay;

        robotEncoderWheelDistance = Double.parseDouble(ReadWriteFile.readFile(wheelBaseSeparationFile).trim()) * CONFIG.COUNTS_PER_INCH;
        this.horizontalEncoderTickPerDegreeOffset = Double.parseDouble(ReadWriteFile.readFile(horizontalTickOffsetFile).trim());
    }

    private void globalCoordinatePositionUpdate() {
        //Get Current Positions
        verticalLeftEncoderWheelPosition = (verticalEncoderLeft.getCurrentPosition() * verticalLeftEncoderPositionMultiplier);
        verticalRightEncoderWheelPosition = (verticalEncoderRight.getCurrentPosition() * verticalRightEncoderPositionMultiplier);

        double leftChange = verticalLeftEncoderWheelPosition - previousVerticalLeftEncoderWheelPosition;
        double rightChange = verticalRightEncoderWheelPosition - previousVerticalRightEncoderWheelPosition;

        //Calculate Angle
        changeInRobotOrientation = (leftChange - rightChange) / (robotEncoderWheelDistance);
        robotOrientationRadians = ((robotOrientationRadians + changeInRobotOrientation));

        //Get the components of the motion
        normalEncoderWheelPosition = (horizontalEncoder.getCurrentPosition()*normalEncoderPositionMultiplier);
        double rawHorizontalChange = normalEncoderWheelPosition - prevNormalEncoderWheelPosition;
        double horizontalChange = rawHorizontalChange - (changeInRobotOrientation*horizontalEncoderTickPerDegreeOffset);

        double p = ((rightChange + leftChange) / 2);
        double n = horizontalChange;

        //Calculate and update the position values
        robotGlobalXCoordinatePosition = robotGlobalXCoordinatePosition + (p*Math.sin(robotOrientationRadians) + n*Math.cos(robotOrientationRadians));
        robotGlobalYCoordinatePosition = robotGlobalYCoordinatePosition + (p*Math.cos(robotOrientationRadians) - n*Math.sin(robotOrientationRadians));

        previousVerticalLeftEncoderWheelPosition = verticalLeftEncoderWheelPosition;
        previousVerticalRightEncoderWheelPosition = verticalRightEncoderWheelPosition;
        prevNormalEncoderWheelPosition = normalEncoderWheelPosition;
    }

    public double getX() { return robotGlobalXCoordinatePosition; }
    public double getXCoordinate(){
        return robotGlobalXCoordinatePosition/ CONFIG.COUNTS_PER_INCH;
    }
    public double getY() { return robotGlobalYCoordinatePosition; }
    public double getYCoordinate(){
        return robotGlobalYCoordinatePosition/ CONFIG.COUNTS_PER_INCH;
    }
    public double getOrientationDegrees() { return Math.toDegrees(robotOrientationRadians) % 360; }
    public double getOrientationRadians(){ return robotOrientationRadians%(Math.PI*2); }

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