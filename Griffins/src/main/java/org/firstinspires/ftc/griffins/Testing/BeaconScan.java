package org.firstinspires.ftc.griffins.Testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.griffins.AutoFunctions;
import org.firstinspires.ftc.griffins.AutoFunctions.DriveStraightDirection;
import org.firstinspires.ftc.griffins.RobotHardware;

/**
 * Created by David on 1/19/2017.
 */
@Autonomous(name = "Beacon Scanning Test", group = "test")
@Disabled
public class BeaconScan extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        RobotHardware hardware = new RobotHardware();
        hardware.initialize(hardwareMap);
        AutoFunctions autoFunctions = new AutoFunctions(hardware, this);
        hardware.registerBeaconColorSensors();

        waitForStart();

        sleep(1000);

        autoFunctions.scanForBeacon(DriveStraightDirection.FORWARD, AutoFunctions.TurnDirection.LEFT);

        sleep(1000);

        //hardware.pushButton(hardware.findBeaconState(), BeaconState.RED);

        sleep(3000);

        //autoFunctions.driveStraightPID(36, DriveStraightDirection.FORWARD, 5);

        //scanForBeacon(DriveStraightDirection.FORWARD, hardware);

        //hardware.pushButton(hardware.findBeaconState(), BeaconState.RED);
    }
}
