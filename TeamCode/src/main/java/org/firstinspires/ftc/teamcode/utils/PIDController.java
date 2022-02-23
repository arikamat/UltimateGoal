package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class PIDController {
    DcMotor motor1,motor2;
    ElapsedTime PIDTimer = new ElapsedTime();
    double kp1, ki1, kd1, kp2,ki2,kd2;
    public PIDController(DcMotor motor, PIDCoefficients coefficients) {
        this.motor1 = motor;
        this.kp1 = coefficients.p;
        this.ki1 =coefficients.i;
        this.kd1 = coefficients.d;

    }
    public PIDController(DcMotor motor1, DcMotor motor2, PIDCoefficients coefficients1,PIDCoefficients coefficients2) {
        this.motor1 = motor1;
        this.motor2 = motor2;
        this.kp1 = coefficients1.p;
        this.ki1 =coefficients1.i;
        this.kd1 = coefficients1.d;
        this.kp2 = coefficients2.p;
        this.ki2 =coefficients2.i;
        this.kd2 = coefficients2.d;

    }
    public void moveToPos(double targetPosition, Telemetry tel){
        PIDTimer.reset();
        double error = this.motor1.getCurrentPosition() - targetPosition;
        double lastError = 0;
        double integral=0;
        double derivative=0;
        while(Math.abs(error)<=9){
            error = this.motor1.getCurrentPosition() - targetPosition;
            double deltaError = lastError-error;
            integral += deltaError * PIDTimer.time();
            derivative = deltaError/PIDTimer.time();
            double P = kp1 *error;
            double I = ki1 *integral;
            double D = kd1 *derivative;
            motor1.setPower(P+I+D);
            PIDTimer.reset();
            tel.addData("Position", motor1.getCurrentPosition());
            tel.update();
        }
    }

}
