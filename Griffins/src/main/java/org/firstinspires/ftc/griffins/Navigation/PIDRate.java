package org.firstinspires.ftc.griffins.Navigation;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.griffins.RobotHardware;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by amandawasserman on 3/4/17.
 */

public class PIDRate {

    private RobotHardware hardware;
    private PIDController pidRateDifference, pidRate;

    private double difference;

    public PIDRate(RobotHardware hardware){
        this.hardware = hardware;
        init();
    }

    public void init() {
        pidRate = new PIDController(1, 0, 0, 5, new Func<Double>() {
            @Override
            public Double value() {
                return (double) (hardware.getShooterLeft().getCurrentPosition() + hardware.getShooterRight().getCurrentPosition()) / 2;
            }
        }, null);
        pidRateDifference = new PIDController(0, 0, 0, 1, new Func<Double>() {
            @Override
            public Double value() {
                return (double) (hardware.getShooterLeft().getCurrentPosition() + hardware.getShooterRight().getCurrentPosition());
            }
        }, null);
    }

    public void setRateTarget(double ecps){
        pidRate.setSetPoint(pidRate.getSourceVal() + ecps);
        pidRateDifference.setSetPoint(pidRateDifference.getSourceVal());
        syncRates();
    }

    public void syncRates(){
        double ecps;

        ecps = pidRate.sendPIDOutput();
        ecps = Range.clip(ecps, -0.5, 0.5);
        difference = pidRateDifference.sendPIDOutput();

        hardware.getShooterLeft().setTargetPosition((int)(ecps + difference));
        hardware.getShooterLeft().setTargetPosition((int)(ecps - difference));
    }

    public String spinToTarget(Func<Boolean> earlyExitCheck, Telemetry telemetry, boolean quickExit) {
        StringBuilder builder = new StringBuilder();
        long lastTime = System.currentTimeMillis();

        int exitValue;
        if (quickExit) {
            exitValue = 1;
        } else {
            exitValue = 100;
        }

        int exitCounter = 0;
        do {
            syncRates();
            String error;
                if (pidRate.isOnTarget()) {
                    exitCounter++;
                } else {
                    exitCounter = 0;
                }

                error = pidRate.getError() + " \n";

            if (System.currentTimeMillis() != lastTime) {
                lastTime = System.currentTimeMillis();
                builder.append(lastTime).append(", ").append(error);
            }
        } while (exitCounter < exitValue && earlyExitCheck.value());


        return builder.toString();
    }

}
