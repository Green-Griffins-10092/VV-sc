package org.firstinspires.ftc.griffins.Testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by David on 11/19/2015.
 * To test servos and motors
 * <p>
 * Controls:
 * Motor:
 * left y axis controls speed
 * dpad up and down switch motor direction
 * left bumper sets run mode to use encoders
 * right bumper sets rum mode to not use encoders
 * start button resets encoders
 * Servo:
 * dpad left and right set servo direction
 * y and b increment position
 * x and a decrement position
 * right y axis changes position
 */

@TeleOp(name = "Motor and Servo Tester", group = "test")
@Disabled
public class MotorServoTest extends OpMode {

    private DcMotor testMotor;
    private Servo testServo;
    private double servoPosition;

    @Override
    public void init() {
        try {
            testMotor = hardwareMap.dcMotor.get("test motor");
        } catch (IllegalArgumentException e) {
            telemetry.addData("Warning", "No test motor!");
        }

        try {
            testServo = hardwareMap.servo.get("test servo");
            double initialPosition = 0;
            testServo.setPosition(initialPosition);
            servoPosition = initialPosition;
        } catch (Exception e) {
            telemetry.addData("Warning", "No test servo!");
        }

        for (DcMotor motor :
                hardwareMap.dcMotor) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            motor.setPower(0);
        }

        //servos do not have a float method
    }

    @Override
    public void loop() {
        //TODO: a button that moves motor very precisely?

        if (testMotor != null) {

            if (gamepad1.right_bumper) {
                testMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            } else if (gamepad1.left_bumper) {
                testMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            } else if (gamepad1.start) {
                testMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }

            testMotor.setPower(-gamepad1.left_stick_y);

            if (gamepad1.dpad_up) {
                testMotor.setDirection(DcMotor.Direction.FORWARD);
            } else if (gamepad1.dpad_down) {
                testMotor.setDirection(DcMotor.Direction.REVERSE);
            }

            telemetry.addData("Motor power", testMotor.getPower());
            telemetry.addData("Motor encoder position", testMotor.getCurrentPosition());
            telemetry.addData("Motor run mode", testMotor.getMode());
            telemetry.addData("Motor direction", testMotor.getDirection());
        }

        if (testServo != null) {
            if (gamepad1.dpad_right) {
                testServo.setDirection(Servo.Direction.FORWARD);
            } else if (gamepad1.dpad_left) {
                testServo.setDirection(Servo.Direction.REVERSE);
            }

            servoPosition = testServo.getPosition();
            //servo adjustments
            if (gamepad1.y) {
                servoPosition += .001;
            } else if (gamepad1.x) {
                servoPosition -= .001;
            } else if (gamepad1.b) {
                servoPosition += .01;
            } else if (gamepad1.a) {
                servoPosition -= .01;
            } else {
                double servoChange = -gamepad1.right_stick_y / 1000;
                servoPosition += servoChange;
            }

            servoPosition = Range.clip(servoPosition, 0, 1);
            testServo.setPosition(servoPosition);
            telemetry.addData("Servo position", servoPosition);
            telemetry.addData("Servo direction", testServo.getDirection());
        }
    }

}