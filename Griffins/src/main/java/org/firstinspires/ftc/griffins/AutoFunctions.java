package org.firstinspires.ftc.griffins;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.griffins.Navigation.LinearOpModeTimeOutFunc;
import org.firstinspires.ftc.griffins.Navigation.PIDDrive;

/**
 * Created by David on 11/28/2016.
 */

public class AutoFunctions {
    public static double[] scanningSpeeds = {0.05, 0.15};
    private LinearOpMode linearOpMode;
    private RobotHardware hardware;
    private PIDDrive drive;

    public AutoFunctions(RobotHardware hardware, LinearOpMode linearOpMode) {
        this.hardware = hardware;
        this.linearOpMode = linearOpMode;
        drive = new PIDDrive(hardware);
    }

    public void wallDrive(double signedPower) {
        double powerRatio = 0.2 / 0.25;

        hardware.setDrivePower(signedPower, signedPower * powerRatio);
    }

    private double determineDrivePower(DriveStraightDirection defaultDirection) {
        RobotHardware.BeaconState beaconState = hardware.findBeaconState();

        double drivePower = 0;

        if (beaconState.containsUndefined()) {
            if (beaconState == RobotHardware.BeaconState.UNDEFINED_UNDEFINED) {
                drivePower = scanningSpeeds[1] * (defaultDirection == DriveStraightDirection.FORWARD ? 1 : -1);

            } else {
                drivePower = scanningSpeeds[0] * (defaultDirection == DriveStraightDirection.FORWARD ? 1 : -1);
            }
        }

        return drivePower;
    }

    public void scanForBeacon(DriveStraightDirection defaultDirection) {
        double drivePower = determineDrivePower(defaultDirection);
        double lastDrivePower = drivePower;

        while (linearOpMode.opModeIsActive() && drivePower != 0) {
            lastDrivePower = drivePower;
            wallDrive(drivePower);
            drivePower = determineDrivePower(defaultDirection);
        }

        hardware.stopDrive();
    }

    public void oneWheelTurn(DcMotor turningMotor, double angle) throws InterruptedException {
        double minimumPower = .2;
        int gyroTarget = (int)getZAngle();

        ElapsedTime timeout = new ElapsedTime();
        double drivePower = 1.0;
        do {
            linearOpMode.idle(); //replace with idle, check that while loops call opmode is active
            int headingError = (gyroTarget - (int)getZAngle());

            RobotLog.i("time: " + timeout.time());
            RobotLog.i("error: " + headingError);
            RobotLog.i("Motor power: " + drivePower);
            RobotLog.i("-----------------------");
            turningMotor.setPower(drivePower);
            drivePower = headingError / (2 * angle);
            drivePower = Range.clip(drivePower, -1, 1);
            if (Math.abs(drivePower) < minimumPower) {
                drivePower = minimumPower * Math.signum(drivePower);
            }
            linearOpMode.telemetry.addData("error", headingError);
            linearOpMode.telemetry.addData("Motor power", drivePower);
            linearOpMode.telemetry.update();
        }
        while (Math.abs(getZAngle()) > 0 && timeout.time() < 5 && linearOpMode.opModeIsActive()); //DAVID: can't find new getRobotRotationGyro method

        //stop driving
        linearOpMode.idle();
        hardware.stopDrive();

        //send any late signals

        linearOpMode.waitForNextHardwareCycle();
        linearOpMode.telemetry.update();
    }

    @Deprecated
    public void twoWheelTurn(double angle) throws InterruptedException {

        //curve around to face the ramp
        int gyroTarget;
        gyroTarget = (int) getZAngle() + (int) angle;

        ElapsedTime timeout = new ElapsedTime();
        double minimumPower = .10;
        double drivePower = 1.0;
        do {
            linearOpMode.idle();
            int headingError = (gyroTarget - (int)getZAngle());
            if (Math.abs(drivePower) < minimumPower) {
                drivePower = minimumPower * Math.signum(drivePower);
            }
            hardware.setDrivePower(drivePower, -drivePower);
            drivePower = -headingError / (9 * Math.abs(angle));
            drivePower = Range.clip(drivePower, -1, 1);
            RobotLog.i("2w -----------------------");
            RobotLog.i("time: " + timeout.time());
            RobotLog.i("error: " + headingError);
            RobotLog.i("Motor power: " + drivePower);
            linearOpMode.telemetry.addData("error", headingError);
            linearOpMode.telemetry.addData("target, current", gyroTarget + ", " + getZAngle());
            linearOpMode.telemetry.addData("Motor power", drivePower);
            linearOpMode.telemetry.update();
        }
        while (Math.abs(getZAngle() - gyroTarget) > 0 && timeout.time() < 5 && linearOpMode.opModeIsActive());

        //stop motors
        linearOpMode.idle();
        hardware.stopDrive();

        //send any late signals
        linearOpMode.waitForNextHardwareCycle();
        linearOpMode.telemetry.update();
    }

    @Deprecated
    public void driveStraight(long encoderCount, DriveStraightDirection direction, double power) throws InterruptedException {
        double minimumPower = .05;
        double maximumPower = power;
        if (power < 0) {
            throw new IllegalArgumentException("Power must be greater than 0");
        } else if (power > 1) {
            throw new IllegalArgumentException("Power must be less than 1");
        }
        if (encoderCount < 0) {
            throw new IllegalArgumentException(" Encoder count must be greater than 0");
        }
        ElapsedTime timeout = new ElapsedTime();
        //encoder target
        long encoderTarget;
        RobotLog.i("Drive Straight --------------");
        RobotLog.i("Direction: " + direction);
        RobotLog.i("Power: " + power);
        RobotLog.i("Encoder Counts: " + encoderCount);

        long leftEncoderOffset = hardware.getLeftDrive().getCurrentPosition();
        long rightEncoderOffset = hardware.getRightDrive().getCurrentPosition();

        if (direction == DriveStraightDirection.BACKWARD) {
            encoderTarget = hardware.getLeftDrive().getCurrentPosition() - encoderCount;
        } else {
            encoderTarget = hardware.getLeftDrive().getCurrentPosition() + encoderCount;
        }
        //drive forward, clearing front of ramp
        timeout.reset();
        boolean stopCondition;
        do {

            //how much the left motor is ahead of the right motor.
            long encoderDifference = (hardware.getLeftDrive().getCurrentPosition() - leftEncoderOffset) - (hardware.getRightDrive().getCurrentPosition() - rightEncoderOffset);
            long error = encoderTarget - (hardware.getLeftDrive().getCurrentPosition() - encoderDifference / 2);

            //calculate using encoder difference
            double powerOffset = encoderDifference / 50;

            if (error != 0) {
                power = error / 50;
            } else {
                power = 0;
            }

            if (Math.abs(power) < minimumPower) {
                power = minimumPower * Math.signum(power);
            } else if (Math.abs(power) > maximumPower) {
                power = maximumPower * Math.signum(power);
            }

            RobotLog.i("DriveStraight loop------");
            RobotLog.i("Encoder Counts to go: " + error);
            RobotLog.i("Power: " + power);
            RobotLog.i("Encoder difference: " + encoderDifference);
            RobotLog.i("Power offset: " + powerOffset);
            linearOpMode.telemetry.addData("Encoder Counts to go", Math.abs(hardware.getLeftDrive().getCurrentPosition() - encoderTarget));

            linearOpMode.idle();
            hardware.setDrivePower(power - powerOffset, power + powerOffset);

            stopCondition = !(Math.abs(hardware.getLeftDrive().getCurrentPosition() - encoderTarget) < RobotHardware.ENCODER_COUNTS_PER_INCH);

            linearOpMode.telemetry.update();
        } while (stopCondition && timeout.time() < 10 && linearOpMode.opModeIsActive());

        //stop motors
        linearOpMode.idle();
        hardware.stopDrive();

        //send any late signals
        linearOpMode.waitForNextHardwareCycle();
        linearOpMode.telemetry.update();

    }

    @Deprecated
    public void driveStraightSimple(long encoderCount, DriveStraightDirection direction, double power) {
        if (power < 0) {
            throw new IllegalArgumentException("Power must be greater than 0");
        } else if (power > 1) {
            throw new IllegalArgumentException("Power must be less than 1");
        }
        if (encoderCount < 0) {
            throw new IllegalArgumentException(" Encoder count must be greater than 0");
        }

        hardware.getLeftDrive().setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hardware.getRightDrive().setMode(DcMotor.RunMode.RUN_TO_POSITION);

        if (direction == DriveStraightDirection.FORWARD) {
            hardware.getLeftDrive().setTargetPosition((int) (encoderCount + hardware.getLeftDrive().getCurrentPosition()));
            hardware.getRightDrive().setTargetPosition((int) (encoderCount + hardware.getRightDrive().getCurrentPosition()));
        } else {
            hardware.getLeftDrive().setTargetPosition((int) (hardware.getLeftDrive().getCurrentPosition() - encoderCount));
            hardware.getRightDrive().setTargetPosition((int) (hardware.getRightDrive().getCurrentPosition() - encoderCount));
        }

        ElapsedTime timeout = new ElapsedTime();
        while (linearOpMode.opModeIsActive() && (hardware.getLeftDrive().isBusy() && hardware.getRightDrive().isBusy()) && timeout.seconds() < 8) {
            hardware.setDrivePower(power, power);
        }

        hardware.stopDrive();

        hardware.getLeftDrive().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hardware.getRightDrive().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void curveDriveShort(long encoderCountLeft, long encoderCountRight, double powerLeft, double powerRight){
        if (powerLeft < -1.0 || powerRight < -1.0){
            throw new IllegalArgumentException("Power must be greater than 0)");
        } else if (powerLeft > 1 || powerRight > 1){
            throw new IllegalArgumentException("Power must be less than 1");
        }

        hardware.getLeftDrive().setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hardware.getRightDrive().setMode(DcMotor.RunMode.RUN_TO_POSITION);

        hardware.getLeftDrive().setTargetPosition((int) (hardware.getLeftDrive().getCurrentPosition() + encoderCountLeft));
        hardware.getRightDrive().setTargetPosition((int) (hardware.getRightDrive().getCurrentPosition() + encoderCountRight));

        ElapsedTime timeout = new ElapsedTime();
        while (linearOpMode.opModeIsActive() && timeout.seconds() < 1) {
            hardware.setDrivePower(powerLeft, powerRight);
        }

        hardware.stopDrive();

        hardware.getLeftDrive().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hardware.getRightDrive().setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void curveDriveLong(long encoderCountLeft, long encoderCountRight, double powerLeft, double powerRight){
        if (powerLeft < 0 || powerRight < 0){
            throw new IllegalArgumentException("Power must be greater than 0)");
        } else if (powerLeft > 1 || powerRight > 1){
            throw new IllegalArgumentException("Power must be less than 1");
        }

        hardware.getLeftDrive().setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hardware.getRightDrive().setMode(DcMotor.RunMode.RUN_TO_POSITION);

        hardware.getLeftDrive().setTargetPosition((int) (hardware.getLeftDrive().getCurrentPosition() - encoderCountLeft));
        hardware.getRightDrive().setTargetPosition((int) (hardware.getRightDrive().getCurrentPosition() - encoderCountRight));

        ElapsedTime timeout = new ElapsedTime();
        while (linearOpMode.opModeIsActive() && timeout.seconds() < 5) {
            hardware.setDrivePower(powerLeft, powerRight);
        }

        hardware.stopDrive();

        hardware.getLeftDrive().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hardware.getRightDrive().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void twoWheelTurnSimple(long encoderCount, TurnDirection direction, double power) {
        if (power < 0) {
            throw new IllegalArgumentException("Power must be greater than 0");
        } else if (power > 1) {
            throw new IllegalArgumentException("Power must be less than 1");
        }
        if (encoderCount < 0) {
            throw new IllegalArgumentException(" Encoder count must be greater than 0");
        }

        hardware.getLeftDrive().setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hardware.getRightDrive().setMode(DcMotor.RunMode.RUN_TO_POSITION);

        if (direction == TurnDirection.LEFT) {
            hardware.getLeftDrive().setTargetPosition((int) (hardware.getLeftDrive().getCurrentPosition() - encoderCount));
            hardware.getRightDrive().setTargetPosition((int) (hardware.getRightDrive().getCurrentPosition() + encoderCount));
        } else {
            hardware.getLeftDrive().setTargetPosition((int) (hardware.getLeftDrive().getCurrentPosition() + encoderCount));
            hardware.getRightDrive().setTargetPosition((int) (hardware.getRightDrive().getCurrentPosition() - encoderCount));
        }

        ElapsedTime timeout = new ElapsedTime();
        while (linearOpMode.opModeIsActive() && (hardware.getLeftDrive().isBusy() && hardware.getRightDrive().isBusy()) && timeout.seconds() < 8) {
            hardware.setDrivePower(power, power);
        }

        hardware.stopDrive();

        hardware.getLeftDrive().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hardware.getRightDrive().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void pushBeacon(RobotHardware.BeaconState alliance) {
        if (linearOpMode.opModeIsActive()) {
            hardware.pushButton(hardware.findBeaconState(), alliance);
            linearOpMode.sleep(1900);
            hardware.pushButton(RobotHardware.BeaconState.UNDEFINED, alliance);
            linearOpMode.sleep(700);
        }
    }

    public void shoot() {
        if (linearOpMode.opModeIsActive()) {
            hardware.getShooter().setPower(0.75);
            linearOpMode.sleep(500);
            hardware.setLoaderPower(1.0);
            linearOpMode.sleep(1000);
            hardware.setLoaderPower(0);
            linearOpMode.sleep(500);
            hardware.setLoaderPower(1);
            linearOpMode.sleep(1000);
            hardware.getShooter().setPower(0.0);
            hardware.setLoaderPower(0.0);
        }
    }

    public float getZAngle(){
        //return hardware.getRobotTracker().getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX).firstAngle;
        return hardware.getTurretGyro().getIntegratedZValue();
    }

    public void driveStraightPID(double inches, DriveStraightDirection direction, double timeoutSeconds, boolean quickExit) {
        drive.setDriveTarget(inches * (direction == DriveStraightDirection.FORWARD ? 1 : -1));
        drive.driveToTarget(new LinearOpModeTimeOutFunc(linearOpMode, timeoutSeconds), linearOpMode.telemetry);
    }

    public void driveStraightPID(double inches, DriveStraightDirection direction, boolean quickExit) {
        driveStraightPID(inches, direction, 8, quickExit);
    }

    public void driveStraightPID(double inches, DriveStraightDirection direction) {
        driveStraightPID(inches, direction, 8, false);
    }

    public void driveStraightPID(double inches, DriveStraightDirection direction, double timeoutSeconds) {
        driveStraightPID(inches, direction, timeoutSeconds, false);
    }

    public String twoWheelTurnPID(double degrees, TurnDirection direction, double timeoutSeconds, boolean quickExit) {
        drive.setTurnTarget(degrees * (direction == TurnDirection.LEFT ? 1 : -1));
        return drive.driveToTarget(new LinearOpModeTimeOutFunc(linearOpMode, timeoutSeconds), linearOpMode.telemetry, quickExit);
    }

    public String twoWheelTurnPID(double degrees, TurnDirection direction, boolean quickExit) {
        return twoWheelTurnPID(degrees, direction, 5, quickExit);
    }

    public String twoWheelTurnPID(double degrees, TurnDirection direction) {
        return twoWheelTurnPID(degrees, direction, 5, false);
    }

    public String twoWheelTurnPID(double degrees, TurnDirection direction, double timeoutSeconds) {
        return twoWheelTurnPID(degrees, direction, timeoutSeconds, false);
    }

    public void wallPIDDrive(double inches, DriveStraightDirection direction, double timeoutSeconds) {
        drive.setDriveTarget(inches * (direction == DriveStraightDirection.FORWARD ? 1 : -1));
        drive.wallDriveToTarget(new LinearOpModeTimeOutFunc(linearOpMode, timeoutSeconds));
    }

    public enum DriveStraightDirection {FORWARD, BACKWARD}

    public enum TurnDirection {RIGHT, LEFT}
}