package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.util.ElapsedTime;

public class SquidController {
    private double kSQ;
    private double kD;
    private double kI;

    ElapsedTime timer = new ElapsedTime();
    private double lastError = 0.0;
    private double integralSum = 0.0;

    public SquidController(double kS) {
        this.kSQ = kS;
    }

    public SquidController(double kS, double kI, double kD){
        this.kSQ = kS;
        this.kI = kI;
        this.kD = kD;
    }

    /**
     *
     *
     * @param error, target - measured, no need to worry about negatives
     * @return the output power
     */
    public double calculate (double error) {
        integralSum += error * timer.seconds();
        double derivative = (error - lastError) / timer.seconds();
        lastError = error;

        timer.reset();

        return Math.sqrt(Math.abs(kSQ)) * Math.signum(error) + derivative * kD + integralSum * kI;
    }

    public double calculate (double sp, double cur) {
        double error = cur - sp;

        integralSum += error * timer.seconds();
        double derivative = (error - lastError) / timer.seconds();
        lastError = error;

        timer.reset();

        return Math.sqrt(Math.abs(kSQ)) * Math.signum(error) + derivative * kD + integralSum * kI;
    }

    public void setkSQ(double kS){
        this.kSQ = kS;
    }

    public void setK(double kS, double kD) {
        this.kSQ = kS;
        this.kD = kD;
    }

}
