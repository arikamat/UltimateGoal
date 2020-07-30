package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Used to interact with the 3 dead-wheel Encoders on the drivetrain for odometry and motion-planning
 *
 */
public class Encoders {
    DcMotor verticalLeft, verticalRight, horizontal;

    /**
     * This accepts the HardWareMap and gets all of the encoders based on the name from the CONFIG file.
     * @param hmap
     */
    public Encoders(HardwareMap hmap){
        this.verticalLeft = hmap.get(DcMotor.class,CONFIG.LEFTVERTICAL);
        this.verticalRight=hmap.get(DcMotor.class,CONFIG.RIGHTVERTICAL);
        this.horizontal = hmap.get(DcMotor.class,CONFIG.HORIZONTAL);

    }

    /**
     * Void Function to apply DcMotor.RunMode.RUN_WITHOUT_ENCODER and DcMotor.RunMode.STOP_AND_RESET_ENCODER
     */
    public void resetEncoders(){
        this.verticalLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.verticalRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.horizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.verticalLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.verticalRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.horizontal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    /**
     * This function doesn't take in any parameters and returns the average value of the 2 forward emcoders that are parallel to the mecanum wheels
     * @return The average ticks of the left and right encoders; intended to get the average movement in the forward direction
     */
    public double getAverageLeftRight(){
        int average = (this.verticalLeft.getCurrentPosition()+this.verticalRight.getCurrentPosition())/2;
        return average;
    }

    /**
     * Function that takes in 0 parameters and returns left vertical encoder's position
     * @return left vertical encoder's position
     */
    public double getLeft(){
        return this.verticalLeft.getCurrentPosition();
    }
    /**
     * Function that takes in 0 parameters and returns right vertical encoder's position
     * @return Right vertical encoder's position
     */
    public double getRight(){
        return this.verticalRight.getCurrentPosition();
    }
    /**
     * Function that takes in 0 parameters and returns horizontal encoder's position
     * @return Horizontal encoder's position
     */
    public double getHorizontal(){
        return this.horizontal.getCurrentPosition();
    }
}
