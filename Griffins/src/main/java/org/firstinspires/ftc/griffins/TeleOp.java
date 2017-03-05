package org.firstinspires.ftc.griffins;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.griffins.RobotHardware.BeaconState;

import static org.firstinspires.ftc.griffins.RobotHardware.BeaconState.BLUE;
import static org.firstinspires.ftc.griffins.RobotHardware.BeaconState.BLUE_RED;
import static org.firstinspires.ftc.griffins.RobotHardware.BeaconState.RED;
import static org.firstinspires.ftc.griffins.RobotHardware.BeaconState.RED_BLUE;
import static org.firstinspires.ftc.griffins.RobotHardware.BeaconState.UNDEFINED;

/**
 * Created by David on 11/26/2016.
 */
public abstract class TeleOp extends OpMode {

    protected BeaconState alliance;
    private RobotHardware hardware;

    @Override
    public void init() {
        hardware = new RobotHardware();
        hardware.initialize(hardwareMap);

        hardware.registerBeaconColorSensors();
        hardware.registerLoaderColorSensor();

        gamepad1.setJoystickDeadzone(0.1f);
        gamepad2.setJoystickDeadzone(0.1f);
    }

    @Override
    public void init_loop() {
        telemetry.addData("Ready for Teleop", !hardware.getTurretGyro().isCalibrating());
    }

    @Override
    public void start() {
        super.start();
        this.resetStartTime();
        hardware.startTurretTracking();
    }

    @Override
    public void loop() {
        double leftDrivePower;
        double rightDrivePower;
        double shooterPower;
        double intakeSpeed;
        double loaderPower;
        double targetTurretSpeed;
        BeaconState beaconPushState;
        double beaconPushRatio;
        boolean turretState;
        String particle;


        { //gamepad 1 controls
            rightDrivePower = Math.pow(-gamepad1.right_stick_y, 3);
            leftDrivePower = Math.pow(-gamepad1.left_stick_y, 3);

            intakeSpeed = gamepad1.left_trigger - gamepad1.right_trigger;

            if (gamepad1.right_bumper || gamepad1.left_bumper) {
                leftDrivePower *= .7;
                rightDrivePower = leftDrivePower * 0.9;
            }

        } //end gamepad 1 controls


        { //gamepad 2 controls
            RobotHardware.BeaconState ball = hardware.findParticleColor();
            if (gamepad2.a) {
                if (ball == alliance) {
                    loaderPower = 1;
                    intakeSpeed = 1;
                } else if (ball == UNDEFINED) {
                    loaderPower = 0;
                } else {
                    loaderPower = -1;
                    intakeSpeed = -1;
                }
            } else {
                if (gamepad2.left_bumper) {
                    loaderPower = -1.0;
                } else if (gamepad2.left_trigger != 0) {
                    loaderPower = 1;
                } else {
                    loaderPower = 0;
                }
            }

            if (ball == RED) {
                particle = "Red Particle";
            } else if (ball == BLUE) {
                particle = "Blue Particle";
            } else {
                particle = "No Particle";
            }

            if (gamepad2.right_bumper) {
                shooterPower = 0.70;
            } else if (gamepad2.right_trigger >= 0.5) {
                shooterPower = 0.75;
            } else if (gamepad2.left_bumper) {
                shooterPower = -0.7;
            } else {
                shooterPower = 0;
            }

            targetTurretSpeed = gamepad2.left_stick_x;
            targetTurretSpeed = Math.pow(targetTurretSpeed, 3);
            if (!gamepad2.left_stick_button) {
                targetTurretSpeed /= 3;
            }
            turretState = false; //!gamepad2.left_stick_button && shooterPower == 0;

            if (gamepad2.x || gamepad2.b || gamepad2.right_stick_button) {
                beaconPushState = hardware.findBeaconState();
                beaconPushRatio = 1;
            } else if (gamepad2.right_stick_x < -0.1) {
                beaconPushState = alliance == RED ? RED_BLUE : BLUE_RED;
                beaconPushRatio = -gamepad2.right_stick_x;
            } else if (gamepad2.right_stick_x > 0.1) {
                beaconPushState = alliance == RED ? BLUE_RED : RED_BLUE;
                beaconPushRatio = gamepad2.right_stick_x;
            } else {
                beaconPushState = BeaconState.UNDEFINED;
                beaconPushRatio = RobotHardware.BUTTON_PUSHER_RATIO;
            }
        } //end gamepad 2 controls


        { //send hardware commands
            hardware.setDrivePower(leftDrivePower, rightDrivePower);
            hardware.getShooter().setPower(shooterPower);
            hardware.getIntake().setPower(intakeSpeed);
            hardware.setLoaderPower(loaderPower);
            hardware.pushButton(beaconPushState, alliance, beaconPushRatio);
            hardware.setTurretRotation(targetTurretSpeed, turretState);
        } //end send hardware commands


        { //send telemetry commands
            int time = (int) getRuntime();
            telemetry.addData("Time(current:remaining)", time + ":" + (120 - time));
            telemetry.addData("Particle Being Loaded", particle);
            telemetry.addData("Left Drive Speed", leftDrivePower);
            telemetry.addData("Right Drive Speed", rightDrivePower);
            telemetry.addData("Intake Speed", intakeSpeed);
            telemetry.addData("Loader Speed", loaderPower);
            telemetry.addData("Shooter Speed", shooterPower);
            telemetry.addData("gamepad 1", gamepad1);
            telemetry.addData("gamepad 2", gamepad2);
            telemetry.addData("left sensor data(a b r g)", hardware.getLeftButtonPusherColorSensor().alpha() + " " +
                    hardware.getLeftButtonPusherColorSensor().blue() + " " + hardware.getLeftButtonPusherColorSensor().red() +
                    " " + hardware.getLeftButtonPusherColorSensor().green());
            telemetry.addData("Right sensor data(a b r g)", hardware.getRightButtonPusherColorSensor().alpha() + " " +
                    hardware.getRightButtonPusherColorSensor().blue() + " " + hardware.getRightButtonPusherColorSensor().red() +
                    " " + hardware.getRightButtonPusherColorSensor().green());
            telemetry.addData("Loader sensor data(a b r g)", hardware.getLoaderColorSensor().alpha() + " " +
                    hardware.getLoaderColorSensor().blue() + " " + hardware.getLoaderColorSensor().red() +
                    " " + hardware.getLoaderColorSensor().green());
        } //end send telemetry commands
    }

    @Override
    public void stop() {
        hardware.pushButton(BeaconState.UNDEFINED);
    }
}
