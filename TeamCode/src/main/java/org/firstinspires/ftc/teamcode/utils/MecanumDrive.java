package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class MecanumDrive {
    private String flName = CONFIG.FRONTLEFT;
    private String frName = CONFIG.FRONTRIGHT;
    private String blName = CONFIG.BACKLEFT;
    private String brName = CONFIG.BACKRIGHT;
    private DcMotor fl, fr,bl,br;
    public MecanumDrive(HardwareMap hardwareMap, boolean usingEncoder){
        this.fl = hardwareMap.get(DcMotor.class, flName);
        this.fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        this.fr = hardwareMap.get(DcMotor.class, frName);
        this.fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        this.bl = hardwareMap.get(DcMotor.class, blName);
        this.bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        this.br = hardwareMap.get(DcMotor.class, brName);
        this.br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if(usingEncoder){
            this.fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            this.fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            this.bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            this.br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        else{
            this.fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            this.fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            this.bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            this.br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }
    public void setRunMode(DcMotor.RunMode runMode){
        this.fl.setMode(runMode);
        this.fr.setMode(runMode);
        this.bl.setMode(runMode);
        this.br.setMode(runMode);
    }
    public void setPower(double fl, double fr, double bl, double br){
        this.fl.setPower(fl);
        this.fr.setPower(fr);
        this.bl.setPower(bl);
        this.br.setPower(br);
    }
    public double[] calculateSpeedsJoysticks(double leftX,double leftY, double rightX){
        double wheelPower, stickAngleRadians, rightx, flPower, frPower, blPower,brPower, sinAngle,cosAngle,factor;
        wheelPower = Math.hypot(leftX,leftY);
        stickAngleRadians= Math.atan2(leftY,leftX);
        stickAngleRadians-=(Math.PI/4.0);
        sinAngle=Math.sin(stickAngleRadians);
        cosAngle= Math.cos(stickAngleRadians);
        factor = 1 / Math.max(Math.abs(sinAngle), Math.abs(cosAngle));
        rightx = rightX*.5;
        flPower = wheelPower * cosAngle * factor + rightX;
        frPower = wheelPower * sinAngle * factor - rightX;
        blPower = wheelPower * sinAngle * factor + rightX;
        brPower = wheelPower * cosAngle * factor - rightX;
        double[] wheelPowers = {flPower,frPower,blPower,brPower};
        return  wheelPowers;
    }
    public void joystickTeleop(Gamepad gamepad1, double speedEnhancer){
        double motorMax=1;
        double X1,X2,Y1,Y2;
        double LF,RF,LR,RR;
        LF=RF=LR=RR=0;
        Y1 = -gamepad1.right_stick_y * speedEnhancer; // invert so up is positive
        X1 = gamepad1.right_stick_x * speedEnhancer;
        Y2 = -gamepad1.left_stick_y * speedEnhancer; // Y2 is not used at present
        X2 = gamepad1.left_stick_x * speedEnhancer;
        // Forward/back movement
        LF += Y1; RF += Y1; LR += Y1; RR += Y1;

        // Side to side movement
        LF += X1; RF -= X1; LR -= X1; RR += X1;

        // Rotation movement
        LF += X2; RF -= X2; LR += X2; RR -= X2;
        LF = Math.max(-motorMax, Math.min(LF, motorMax));
        RF = Math.max(-motorMax, Math.min(RF, motorMax));
        LR = Math.max(-motorMax, Math.min(LR, motorMax));
        RR = Math.max(-motorMax, Math.min(RR, motorMax));

        // Send values to the motors
        this.fl.setPower(LF);
        this.fr.setPower(RF);
        this.bl.setPower(LR);
        this.br.setPower(RR);
    }
    public void stop(){
        setPower(0.0,0.0,0.0,0.0);
    }
}
