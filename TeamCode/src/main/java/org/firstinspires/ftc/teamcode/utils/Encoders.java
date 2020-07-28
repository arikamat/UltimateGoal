package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Encoders {
    DcMotor verticalLeft, verticalRight, horizontal;
    public Encoders(HardwareMap hmap){
        this.verticalLeft = hmap.get(DcMotor.class,CONFIG.LEFTVERTICAL);
        this.verticalRight=hmap.get(DcMotor.class,CONFIG.RIGHTVERTICAL);
        this.horizontal = hmap.get(DcMotor.class,CONFIG.HORIZONTAL);

    }
    public void resetEncoders(){
        this.verticalLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.verticalRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.horizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.verticalLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.verticalRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.horizontal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }
    public double getAverageLeftRight(){
        int average = (this.verticalLeft.getCurrentPosition()+this.verticalRight.getCurrentPosition())/2;
        return average;
    }
    public double getLeft(){
        return this.verticalLeft.getCurrentPosition();
    }
    public double getRight(){
        return this.verticalRight.getCurrentPosition();
    }
    public double getHorizontal(){
        return this.horizontal.getCurrentPosition();
    }
}
