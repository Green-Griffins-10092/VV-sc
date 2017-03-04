package org.firstinspires.ftc.griffins.Testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.griffins.AutoFunctions;
import org.firstinspires.ftc.griffins.Navigation.LinearOpModeTimeOutFunc;
import org.firstinspires.ftc.griffins.RobotHardware;

/**
 * Created by David on 2/11/2017.
 */

@Autonomous(name = "Registering color sensors test", group = "test")
@Disabled
public class RegisterColorSensors extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        RobotHardware hardware = new RobotHardware();
        hardware.initialize(hardwareMap);
        AutoFunctions autoFunctions = new AutoFunctions(hardware, this);

        waitForStart();

        hardware.registerBeaconColorSensors();

        LinearOpModeTimeOutFunc timeOutFunc = new LinearOpModeTimeOutFunc(this, 2);
        while (timeOutFunc.value()) {
            telemetry.addData("color sensor data", Integer.toHexString(hardware.getLeftButtonPusherColorSensor().argb()));
            telemetry.addData("color sensor data", Integer.toHexString(hardware.getRightButtonPusherColorSensor().argb()));
            telemetry.update();
        }

        telemetry.update();
        sleep(1000);

        hardware.deregisterBeaconColorSensors();

        timeOutFunc = new LinearOpModeTimeOutFunc(this, 2);
        while (timeOutFunc.value()) {
            telemetry.addData("color sensor data", Integer.toHexString(hardware.getLeftButtonPusherColorSensor().argb()));
            telemetry.addData("color sensor data", Integer.toHexString(hardware.getRightButtonPusherColorSensor().argb()));
            telemetry.update();
        }

        telemetry.update();
        sleep(1000);

        hardware.registerBeaconColorSensors();

        timeOutFunc = new LinearOpModeTimeOutFunc(this, 5);
        while (timeOutFunc.value()) {
            telemetry.addData("color sensor data", Integer.toHexString(hardware.getLeftButtonPusherColorSensor().argb()));
            telemetry.addData("color sensor data", Integer.toHexString(hardware.getRightButtonPusherColorSensor().argb()));
            telemetry.update();
        }
    }
}
