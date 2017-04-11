package org.firstinspires.ftc.griffins;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.griffins.Navigation.LinearOpModeTimeOutFunc;
import org.firstinspires.ftc.griffins.Navigation.PIDDrive;
import org.firstinspires.ftc.griffins.Navigation.PIDRate;
import org.firstinspires.ftc.griffins.RobotHardware.BeaconState;
import org.firstinspires.ftc.robotcore.external.Func;

import static org.firstinspires.ftc.griffins.RobotHardware.BUTTON_PUSHER_RATIO;
import static org.firstinspires.ftc.griffins.RobotHardware.BeaconState.BLUE;
import static org.firstinspires.ftc.griffins.RobotHardware.BeaconState.BLUE_BLUE;
import static org.firstinspires.ftc.griffins.RobotHardware.BeaconState.BLUE_RED;
import static org.firstinspires.ftc.griffins.RobotHardware.BeaconState.RED;
import static org.firstinspires.ftc.griffins.RobotHardware.BeaconState.RED_BLUE;
import static org.firstinspires.ftc.griffins.RobotHardware.BeaconState.RED_RED;
import static org.firstinspires.ftc.griffins.RobotHardware.BeaconState.guessBeaconState;

/**
 * Created by David on 11/28/2016.
 */

public class AutoFunctions {
    public static final double[] scanningSpeeds = {0.07, 0.15};
    public static final double SHOOTING_SPEED = 0.765;

    private LinearOpMode linearOpMode;
    private RobotHardware hardware;
    private PIDDrive drive;
    private PIDRate rate;
    private BeaconState alliance;
    //AutoLoadTimeOutFunc 'static' variables
    private ElapsedTime reverseTimer = new ElapsedTime(0);
    private ElapsedTime loaderTimer = new ElapsedTime(0);

    public AutoFunctions(RobotHardware hardware, LinearOpMode linearOpMode) {
        this(hardware, linearOpMode, null);
    }

    public AutoFunctions(RobotHardware hardware, LinearOpMode linearOpMode, BeaconState alliance) {
        this.hardware = hardware;
        this.linearOpMode = linearOpMode;
        drive = new PIDDrive(hardware);
        rate = new PIDRate(hardware);
        this.alliance = alliance;
    }

    public BeaconState getAlliance() {
        return alliance;
    }

    public void setAlliance(BeaconState alliance) {
        this.alliance = alliance;
    }

    public void wallDrive(double signedPower, TurnDirection turnDirection) {
        double powerRatio = 0.8;

        if (turnDirection == TurnDirection.RIGHT) {
            hardware.setDrivePower(signedPower, signedPower * powerRatio);
            linearOpMode.telemetry.addData("Power L:R", signedPower + ":" + signedPower * powerRatio);
        } else if (turnDirection == TurnDirection.LEFT) {
            hardware.setDrivePower(signedPower * powerRatio, signedPower);
            linearOpMode.telemetry.addData("Power L:R", signedPower * powerRatio + ":" + signedPower);

        }
    }

    private double determineDrivePower(DriveStraightDirection defaultDirection, TurnDirection turnDirection) {
        BeaconState beaconState;
        if (turnDirection == TurnDirection.RIGHT) {
            beaconState = hardware.findRightBeaconState();
        } else {
            beaconState = hardware.findLeftBeaconState();
        }

        linearOpMode.telemetry.addData("Beacon State", beaconState);

        double drivePower = 0;

        if (beaconState.containsUndefined()) {
            if (beaconState == BeaconState.UNDEFINED_UNDEFINED) {
                drivePower = scanningSpeeds[1] * (defaultDirection == DriveStraightDirection.FORWARD ? 1 : -1.1);
            } else {

                /*if (beaconState.getBackState() == BeaconState.UNDEFINED) {
                    defaultDirection = DriveStraightDirection.FORWARD;
                } else {
                    defaultDirection = DriveStraightDirection.BACKWARD;
                }*/

                drivePower = scanningSpeeds[0] * (defaultDirection == DriveStraightDirection.FORWARD ? 1 : -1.1);
            }
        }

        linearOpMode.telemetry.addData("Drive Power", drivePower);

        return drivePower;
    }

    public void scanForBeacon(DriveStraightDirection defaultDirection, TurnDirection turnDirection) {
        double drivePower = determineDrivePower(defaultDirection, turnDirection);

        Func<Boolean> timeout = new AutoLoadTimeOutFunc(linearOpMode, 15);

        while (timeout.value() && drivePower != 0) {
            wallDrive(drivePower, turnDirection);
            drivePower = determineDrivePower(defaultDirection, turnDirection);
            linearOpMode.telemetry.update();
        }

        hardware.setLoaderPower(0);
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

    public void pushBeacon(BeaconState beaconState, BeaconState alliance) {
        pushBeacon(beaconState, alliance, false);
    }

    public void pushBeacon(BeaconState beaconState, BeaconState alliance, boolean shoot) {
        if (linearOpMode.opModeIsActive()) {
            beaconState = guessBeaconState(beaconState);
            double inchesBetweenButtons = 4.5;

            if (shoot) {
                hardware.getShooter().setPower(SHOOTING_SPEED);
            }

            if ((alliance == BLUE && beaconState == RED_BLUE) || (alliance == RED && beaconState == BLUE_RED)) {
                wallPIDDrive(inchesBetweenButtons, DriveStraightDirection.FORWARD, alliance == BLUE ? TurnDirection.RIGHT : TurnDirection.LEFT, 1);
            }

            if (shoot) {
                autoLoadingSleep(500);
                hardware.setLoaderPower(1);
            }

            if (!((alliance == BLUE && beaconState == BLUE_BLUE) || (alliance == RED && beaconState == RED_RED))) {
                hardware.extendButtonPusher(BUTTON_PUSHER_RATIO);
                if (shoot) {
                    linearOpMode.sleep(2000);
                } else {
                    autoLoadingSleep(2000);
                }
            }


            hardware.retractButtonPusher();
            if (shoot) {
                linearOpMode.sleep(1000);
            } else {
                autoLoadingSleep(1000);
            }
        }
    }

    public void pushBeacon(BeaconState alliance) {
        pushBeacon(alliance, false);
    }

    public void pushBeacon(BeaconState alliance, boolean shoot) {
        RobotHardware.BeaconState beaconState;
        if (alliance == BLUE) {
            beaconState = hardware.findRightBeaconState();
        } else {
            beaconState = hardware.findLeftBeaconState();
        }

        pushBeacon(beaconState, alliance, shoot);
    }

    public void pushBeacon() {
        pushBeacon(alliance);
    }

    public void shoot(){
        if (linearOpMode.opModeIsActive()){
            hardware.getShooter().setPower(SHOOTING_SPEED);
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

    public void shootPID() {
        if (linearOpMode.opModeIsActive()) {
            rate.setRateTarget(7);
            linearOpMode.sleep(500);
            hardware.setLoaderPower(1.0);
            linearOpMode.sleep(2000);
            hardware.setLoaderPower(0.0);
            linearOpMode.sleep(500);
            rate.setRateTarget(0);
            hardware.setLoaderPower(0.0);
        }
    }

    public void autoLoad(BeaconState alliance) {
        double loaderPower;
        double intakePower = 1.0;

        BeaconState ball = hardware.findParticleColor();

        if (ball == alliance) {
            loaderPower = 1;
            intakePower = 1;
        } else if (ball != alliance) {
            loaderPower = 0.0;
        } else {
            loaderPower = -1;
            intakePower = -1;
        }

        hardware.getIntake().setPower(intakePower);
        hardware.setLoaderPower(loaderPower);

    }

    public float getZAngle(){
        //return hardware.getRobotTracker().getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX).firstAngle;
        return hardware.getTurretGyro().getIntegratedZValue();
    }

    public void driveStraightPID(double inches, DriveStraightDirection direction, double timeoutSeconds, boolean quickExit) {
        drive.setDriveTarget(inches * (direction == DriveStraightDirection.FORWARD ? 1 : -1));
        drive.driveToTarget(new AutoLoadTimeOutFunc(linearOpMode, timeoutSeconds), linearOpMode.telemetry);
        hardware.setLoaderPower(0);
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
        String data = drive.driveToTarget(new AutoLoadTimeOutFunc(linearOpMode, timeoutSeconds), linearOpMode.telemetry, quickExit);
        hardware.setLoaderPower(0);
        return data;
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

    public String shootPIDtoString(double rps) {
        return rate.spinToTarget(new LinearOpModeTimeOutFunc(linearOpMode, 10), linearOpMode.telemetry, false);
    }

    public void wallPIDDrive(double inches, DriveStraightDirection direction, TurnDirection turnDirection, double timeoutSeconds) {
        drive.setDriveTarget(inches * (direction == DriveStraightDirection.FORWARD ? 1 : -1));
        double leftBias = 0, rightBias = 0;
        if (turnDirection == TurnDirection.RIGHT) {
            leftBias = 1;
            rightBias = .8;
        } else if (turnDirection == TurnDirection.LEFT) {
            leftBias = .8;
            rightBias = 1;
        }
        drive.wallDriveToTarget(leftBias, rightBias, new AutoLoadTimeOutFunc(linearOpMode, timeoutSeconds));
        hardware.setLoaderPower(0);
    }

    public void autoLoadingSleep(int milliseconds) {
        AutoLoadTimeOutFunc timeOutFunc = new AutoLoadTimeOutFunc(linearOpMode, milliseconds / 1000.0);
        while (timeOutFunc.value())
            linearOpMode.telemetry.update();
        hardware.setLoaderPower(0);
    }

    public void AutoLoadingTest() {
        AutoLoadTimeOutFunc autoLoadTimeOutFunc = new AutoLoadTimeOutFunc(linearOpMode, 30);

        while (autoLoadTimeOutFunc.value())
            linearOpMode.telemetry.update();
    }

    public enum DriveStraightDirection {FORWARD, BACKWARD}
    public enum TurnDirection {RIGHT, LEFT}

    public class AutoLoadTimeOutFunc extends LinearOpModeTimeOutFunc {

        public AutoLoadTimeOutFunc(LinearOpMode opMode, double timeOutLengthSeconds) {
            super(opMode, timeOutLengthSeconds);
            hardware.registerLoaderColorSensor();
        }

        @Override
        public Boolean value() {

            if (alliance != null) {
                double loaderPower = loaderTimer.milliseconds() > 200 ? 0 : .5;
                double intakePower = reverseTimer.milliseconds() > 500 ? 1 : -1;

                BeaconState ball = hardware.findParticleColor();

                if (ball == alliance) { // particle is our color
                    loaderPower = 1;
                    intakePower = 1;
                    loaderTimer.reset();
                } else if (!ball.containsUndefined()) { // particle is not our color
                    loaderPower = -1;
                    intakePower = -1;
                    reverseTimer.reset();
                    loaderTimer.reset();
                }

                hardware.getIntake().setPower(intakePower);
                hardware.setLoaderPower(loaderPower);

                linearOpMode.telemetry.addData("Particle Color", ball);
                linearOpMode.telemetry.addData("Loader Power", loaderPower);
                linearOpMode.telemetry.addData("Intake Power", intakePower);
                linearOpMode.telemetry.addData("Timer Value", reverseTimer.milliseconds());

            }


            return super.value();
        }
    }
}