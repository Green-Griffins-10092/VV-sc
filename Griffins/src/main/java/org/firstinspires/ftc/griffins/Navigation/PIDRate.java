package org.firstinspires.ftc.griffins.Navigation;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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
        pidRate = new PIDController(.1, 0, 0, .08, 5, new Func<Double>() {
            @Override
            public Double value() {
                return (double) (hardware.getShooterLeft().getCurrentPosition() + hardware.getShooterRight().getCurrentPosition()) / 2;
            }
        }, hardware.getShooter());
        pidRateDifference = new PIDController(0, 0, 0, 1, new Func<Double>() {
            @Override
            public Double value() {
                return (double) (hardware.getShooterLeft().getCurrentPosition() + hardware.getShooterRight().getCurrentPosition());
            }
        }, null);
    }

    public void setRateTarget(double ecps){
        pidRate.setSetPoint(ecps);
        hardware.getShooterLeft().setPower(-pidRate.sendPIDOutput());
        hardware.getShooterRight().setPower(pidRate.sendPIDOutput());
        pidRateDifference.setSetPoint(pidRateDifference.getSourceVal());
    }

    public void syncRates(){
        double ecps;

        ecps = pidRate.sendPIDOutput();
        ecps = Range.clip(ecps, -0.5, 0.5);
        difference = pidRateDifference.sendPIDOutput();

        hardware.getShooterLeft().setTargetPosition((int)(ecps + difference));
        hardware.getShooterRight().setTargetPosition((int)(ecps - difference));
    }

    public String spinToTarget(Func<Boolean> earlyExitCheck, Telemetry telemetry, boolean quickExit) {
        StringBuilder builder = new StringBuilder();
        long lastTime = System.currentTimeMillis();
        int lastDist = hardware.getShooter().getCurrentPosition();

        int exitValue;
        if (quickExit) {
            exitValue = 1;
        } else {
            exitValue = 100;
        }

        int exitCounter = 0;
        //do {
        setRateTarget(7);
        String error;
        while (exitCounter < exitValue && earlyExitCheck.value()) {
                if (pidRate.isOnTarget()) {
                    exitCounter++;
                } else {
                    exitCounter = 0;
                }

            error = hardware.getShooter().getCurrentPosition() + " \n";

            long time = System.currentTimeMillis();
            int dist = hardware.getShooter().getCurrentPosition();

            long changeTime = time - lastTime;
            int changeDist = dist - lastDist;

            double rate = changeDist / changeTime;

            builder.append(time).append(", ").append(rate);

            lastTime = time;
            lastDist = dist;

            try {
                Thread.sleep(5);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
//            if (System.currentTimeMillis() != lastTime) {
//                lastTime = System.currentTimeMillis();
//                builder.append(lastTime).append(", ").append(error);
//            }
           // }
        }

        return builder.toString();
    }

}

