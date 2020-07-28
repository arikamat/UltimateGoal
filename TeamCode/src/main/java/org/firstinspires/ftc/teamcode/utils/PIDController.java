package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class PIDController {
    DcMotor motor;
    ElapsedTime PIDTimer = new ElapsedTime();
    double kp,ki,kd;
    public PIDController(DcMotor motor, PIDCoefficients coefficients) {
        this.motor = motor;
        this.kp = coefficients.p;
        this.ki =coefficients.i;
        this.kd = coefficients.d;

    }
    public void moveToPos(double targetPosition, Telemetry tel){
        PIDTimer.reset();
        double error = this.motor.getCurrentPosition() - targetPosition;
        double lastError = 0;
        double integral=0;
        double derivative=0;
        while(Math.abs(error)<=9){
            error = this.motor.getCurrentPosition() - targetPosition;
            double deltaError = lastError-error;
            integral += deltaError * PIDTimer.time();
            derivative = deltaError/PIDTimer.time();
            double P = kp*error;
            double I = ki*integral;
            double D = kd*derivative;
            motor.setPower(P+I+D);
            PIDTimer.reset();
            tel.addData("Position", motor.getCurrentPosition());
            tel.update();
        }
    }

}
