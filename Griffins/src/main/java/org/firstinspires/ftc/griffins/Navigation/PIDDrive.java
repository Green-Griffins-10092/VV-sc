package org.firstinspires.ftc.griffins.Navigation;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.griffins.RobotHardware;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import static org.firstinspires.ftc.griffins.RobotHardware.ENCODER_COUNTS_PER_ROBOT_DEGREE;

/**
 * Created by David on 12/19/2016.
 */

public class PIDDrive {

    private RobotHardware hardware;
    private PIDController pidDrivingDifference;
    private PIDController pidTurningDifference;

    private double difference;
    private boolean isTurning;

    private PIDController pidTurning, pidDrive;

    public PIDDrive(RobotHardware hardware) {
        this.hardware = hardware;
        init();
    }

    public void init(){
        pidDrive = new PIDController(0.0015, 0, 0.003, 22.3, new Func<Double>() {
            @Override
            public Double value() {
                return (double) (hardware.getLeftDrive().getCurrentPosition() + hardware.getRightDrive().getCurrentPosition()) / 2;
            }
        }, null);

        pidTurning = new PIDController(0.01, 0.0000075, 0.05, 1, new Func<Double>() { //i = .0025
            @Override
            public Double value() {
                return (double) hardware.getTurretGyro().getIntegratedZValue();
            }
        }, null);

        pidDrivingDifference = new PIDController(0.001 * ENCODER_COUNTS_PER_ROBOT_DEGREE, 0, 0, 0, new Func<Double>() {
            @Override
            public Double value() {
                return (double) -hardware.getTurretGyro().getIntegratedZValue();
            }
        }, null);

        pidTurningDifference = new PIDController(0, 0, 0, 22.3, new Func<Double>() {
            @Override
            public Double value() {
                return (double) (hardware.getLeftDrive().getCurrentPosition() + hardware.getRightDrive().getCurrentPosition());
            }
        }, null);
    }

    public void syncDrives(){
        double power;

        if (isTurning) {
            power = pidTurning.sendPIDOutput();
            power = Range.clip(power, -0.4, 0.4);
            difference = pidTurningDifference.sendPIDOutput();

            hardware.setDrivePower(-power - difference, power + difference);
        } else {
            power = pidDrive.sendPIDOutput();
            power = Range.clip(power, -0.5, 0.5);
            difference = pidDrivingDifference.sendPIDOutput();

            hardware.setDrivePower(power + difference, power - difference);
        }
    }

    public void biasedSyncDrives() {
        if (!isTurning) {
            double power;
            power = pidDrive.sendPIDOutput();
            power = Range.clip(power, -0.5, 0.5);
            difference = Range.clip(pidDrivingDifference.sendPIDOutput(), 0, 0.5);

            hardware.setDrivePower(power, power * 0.90);
        }
    }

    public void setDriveTarget(double inches){
        pidDrive.setSetPoint(pidDrive.getSourceVal() + inches * RobotHardware.ENCODER_COUNTS_PER_INCH);
        pidDrivingDifference.setSetPoint(pidDrivingDifference.getSourceVal());
        isTurning = false;
    }

    //turns to the left are positive...
    public void setTurnTarget(double degrees) {
        pidTurning.setSetPoint(pidTurning.getSourceVal() + degrees);
        pidTurningDifference.setSetPoint(pidTurningDifference.getSourceVal());
        isTurning = true;
    }

    public void wallDriveToTarget(Func<Boolean> earlyExitCheck) {
        if (!isTurning) {
            do {
                biasedSyncDrives();
            } while (earlyExitCheck.value() && !pidDrive.isOnTarget());
        }

    }

    public String driveToTarget(Func<Boolean> earlyExitCheck, Telemetry telemetry, boolean quickExit) {
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
            syncDrives();
            String error;
            if (isTurning) {
                if (pidTurning.isOnTarget()) {
                    exitCounter++;
                } else {
                    exitCounter = 0;
                }

                error = pidTurning.getError() + " \n";
            } else {
                if (pidDrive.isOnTarget()) {
                    exitCounter++;
                } else {
                    exitCounter = 0;
                }
                error = pidDrive.getError() + " \n";
            }

            if (System.currentTimeMillis() != lastTime) {
                lastTime = System.currentTimeMillis();
                builder.append(lastTime).append(", ").append(error);
            }

            if (telemetry != null) {
                telemetry.addData("exit counter", exitCounter);
                telemetry.addData("exit condition value", exitCounter <= 10 && earlyExitCheck.value());
                telemetry.addData("source value", isTurning ? pidTurning.getSourceVal() : pidDrive.getSourceVal());
                telemetry.addData("target", isTurning ? pidTurning.getSetPoint() : pidDrive.getSetPoint());
                telemetry.addData("error", isTurning ? pidTurning.getError() : pidDrive.getError());
                telemetry.update();
            }
        } while (exitCounter < exitValue && earlyExitCheck.value());

        hardware.stopDrive();

        if (telemetry != null) {
            telemetry.log().add("exit pid " + (isTurning ? "turn" : "drive") + ", error:" + (isTurning ? pidTurning : pidDrive).getError());
        }

        return builder.toString();
    }

    public String driveToTarget(Func<Boolean> booleanFunc, boolean quickExit) {
        return driveToTarget(booleanFunc, null, quickExit);
    }

    public String driveToTarget(Func<Boolean> booleanFunc, Telemetry telemetry) {
        return driveToTarget(booleanFunc, telemetry, false);
    }

    public String driveToTarget(Func<Boolean> booleanFunc) {
        return driveToTarget(booleanFunc, null, false);
    }
}