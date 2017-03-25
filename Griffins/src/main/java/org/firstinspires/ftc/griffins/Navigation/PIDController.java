package org.firstinspires.ftc.griffins.Navigation;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Func;

/**
 * Created by David on 12/17/2016.
 */

public class PIDController { //for upcoming comp, just use P and D controllers

    private double kP, kI, kD, kF; // proportional gain, integral gain, derivative gain, feed-forward constant
    private double setPoint; // desired value (e.g. encoder count, gyro heading, etc)
    private double sensorValue; // measured value
    private double error;
    private double lastError;
    private double propTerm;
    private double intTerm;
    private double derTerm;
    private double fTerm;
    private Func<Double> source;
    private DcMotor output;
    private boolean isEnabled = false;
    private double tolerance;

    public PIDController(double kP, double kI, double kD, double kF, double tolerance, Func<Double> source, DcMotor output) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
        this.tolerance = tolerance;
        this.setPoint = source.value();
        this.source = source;
        this.output = output;
        lastError = 0.0;
    }

    public PIDController(double kP, double kI, double kD, double tolerance, Func<Double> source, DcMotor output) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = 0;
        this.tolerance = tolerance;
        this.setPoint = source.value();
        this.source = source;
        lastError = 0.0;
    }

    public double getTolerance() {
        return tolerance;
    }

    public void setTolerance(double tolerance) {
        this.tolerance = tolerance;
    }

    public boolean isOnTarget(){
        return Math.abs(error) <= tolerance;
    }

    public double getSourceVal(){
        return source.value();
    }

    public double sendPIDOutput(){
        isEnabled = true;

        calculate();

        double control = propTerm+intTerm+derTerm+fTerm;
        if(output != null)
            output.setPower(control);
        return control;
    }

    public void PIDdisable(){
        isEnabled = false;
        intTerm = 0.0;
        derTerm = 0.0;
    }

    public double getSetPoint(){
        return setPoint;
    }

    public void setSetPoint(double setPoint) {
        this.setPoint = setPoint;
        intTerm = 0;
    }

    public double getError(){
        return error;
    }

    public void calculate(){
        double intError;


        if (isEnabled){
            sensorValue = source.value();
            error = setPoint - sensorValue;
            propTerm = kP*error;
            intError = kI * error;

            if (this.isOnTarget()) {
                intTerm = 0;
            } else if (Math.abs(error) < 10) {
                intTerm += intError;
            }

            if (lastError != 0.0)
                derTerm = kD * (error - lastError);
            else
                derTerm = 0;

            lastError = error;

            fTerm = kF*setPoint;

        }
    }


}