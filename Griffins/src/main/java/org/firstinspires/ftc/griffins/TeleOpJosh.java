package org.firstinspires.ftc.griffins;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.griffins.RobotHardware.BeaconState;

/**
 * Created by David on 1/20/2017.
 */
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Josh Teleop", group = "test")
@Disabled
public class TeleOpJosh extends OpMode {

    private RobotHardware hardware;
    private boolean turretTrackingOn;

    @Override
    public void init() {
        hardware = new RobotHardware();
        hardware.initialize(hardwareMap);

        gamepad1.setJoystickDeadzone(0.1f);
        gamepad2.setJoystickDeadzone(0.1f);
    }

    @Override
    public void init_loop() {
        telemetry.addData("Ready for Teleop", hardware.getTurretGyro().isCalibrating());
    }

    @Override
    public void start() {
        super.start();
        hardware.startTurretTracking();
        turretTrackingOn = false;
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

        rightDrivePower = Math.pow(-gamepad1.right_stick_y, 3);
        leftDrivePower = Math.pow(-gamepad1.left_stick_y, 3);

        intakeSpeed = gamepad1.left_trigger - gamepad1.right_trigger;

        if (gamepad1.right_bumper) {
            rightDrivePower *= .40;
            leftDrivePower *= .40;
        }

        if (gamepad1.left_bumper) {
            rightDrivePower = .20;
            leftDrivePower = .20;
        }

        if (gamepad2.left_bumper) {
            loaderPower = 1.0;
        } else if (gamepad2.left_trigger != 0) {
            loaderPower = -1;
        } else {
            loaderPower = 0;
        }

        if (gamepad2.right_bumper) {
            shooterPower = 0.75;
        } else if (gamepad2.left_trigger > 0.5) {
            shooterPower = -0.7;
        } else {
            shooterPower = 0;
        }

        double currentTurretSpeed = hardware.getTurretRotation().getPower();
        targetTurretSpeed = -gamepad2.left_stick_x;
        /*if (Math.abs(targetTurretSpeed) < 0.2) {
            currentTurretSpeed = targetTurretSpeed;
        } else {
            if (Math.signum(targetTurretSpeed) != Math.signum(currentTurretSpeed)) {
                currentTurretSpeed = 0;
            }

            if (Math.abs(currentTurretSpeed - targetTurretSpeed) <= 0.05) {
                currentTurretSpeed = targetTurretSpeed;
            } else {
                currentTurretSpeed = currentTurretSpeed + Math.signum(targetTurretSpeed - currentTurretSpeed) * 0.05;
            }
        }*/
        targetTurretSpeed = Math.pow(targetTurretSpeed, 3);

        if (gamepad2.right_trigger == 1.0) {
            beaconPushState = BeaconState.BLUE_RED;
        } else {
            beaconPushState = BeaconState.UNDEFINED;
        }

        hardware.setDrivePower(leftDrivePower, rightDrivePower);
        hardware.getShooterLeft().setPower(shooterPower);
        hardware.getShooterRight().setPower(shooterPower);
        hardware.getIntake().setPower(intakeSpeed);
        hardware.setLoaderPower(loaderPower);
        hardware.setTurretRotation(targetTurretSpeed, turretTrackingOn);

        telemetry.addData("Gyro Heading", hardware.getTurretGyro().getIntegratedZValue());


        telemetry.addData("Left Drive Speed", leftDrivePower);
        telemetry.addData("Right Drive Speed", rightDrivePower);
        telemetry.addData("Intake Speed", intakeSpeed);
        telemetry.addData("Loader Speed", loaderPower);
        telemetry.addData("Shooter Speed", shooterPower);
        telemetry.addData("gamepad 1", gamepad1);
        telemetry.addData("gamepad 2", gamepad2);
    }
}