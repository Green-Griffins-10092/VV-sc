package org.firstinspires.ftc.griffins.Testing;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.griffins.RobotHardware;

import static org.firstinspires.ftc.griffins.RobotHardware.BeaconState.RED;
import static org.firstinspires.ftc.griffins.RobotHardware.BeaconState.UNDEFINED;

/**
 * Created by David on 2/11/2017.
 */

@TeleOp(group = "test")
@Disabled
public class IntakeColorSensorTest extends OpMode {

    ModernRoboticsI2cColorSensor colorSensor;
    RobotHardware robotHardware = new RobotHardware();
    RobotHardware.BeaconState alliance = RED;

    @Override
    public void init() {
        robotHardware.initialize(hardwareMap);
        robotHardware.registerLoaderColorSensor();
        colorSensor = robotHardware.getLoaderColorSensor();
    }

    @Override
    public void loop() {
        double loaderPower;
        double intakePower = gamepad1.left_bumper ? -1 : gamepad1.left_trigger;

        RobotHardware.BeaconState ball = robotHardware.findParticleColor();

        if (ball == alliance) {
            loaderPower = 1;
            intakePower = 1;
        } else if (ball == UNDEFINED) {
            loaderPower = gamepad1.right_bumper ? -1 : gamepad1.right_trigger;
        } else {
            loaderPower = -1;
            intakePower = -1;
        }

        robotHardware.getIntake().setPower(intakePower);
        robotHardware.setDrivePower(-gamepad1.left_stick_y, -gamepad1.right_stick_y);
        robotHardware.setLoaderPower(loaderPower);

        telemetry.addData("color sensor sees", robotHardware.findParticleColor());
        telemetry.addData("Red, blue, green, alpha", colorSensor.red() + ", " + colorSensor.blue() + ", " + colorSensor.green()
                + ", " + colorSensor.alpha());
        telemetry.addData("Color Number", robotHardware.getLoaderColorNumber());
    }
}
