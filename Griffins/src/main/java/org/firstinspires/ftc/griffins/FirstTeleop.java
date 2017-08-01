package org.firstinspires.ftc.griffins;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

/**
 * Created by dflor on 7/10/2017.
 */

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class FirstTeleop extends OpMode {

    RobotHardware robot;

    @Override
    public void init() {
        robot = new RobotHardware();
        robot.initialize(hardwareMap);
    }

    @Override
    public void loop() {

        double left;
        double right;
        double intake;

        left = -gamepad1.left_stick_y;
        right = -gamepad1.right_stick_y;
        intake = gamepad1.left_trigger;

        robot.getLeftDrive().setPower(left);
        robot.getRightDrive().setPower(right);
        robot.getIntake().setPower(intake);

        telemetry.addData("left", left);
        telemetry.addData("right", right);
        telemetry.addData("intake", intake);
    }

}