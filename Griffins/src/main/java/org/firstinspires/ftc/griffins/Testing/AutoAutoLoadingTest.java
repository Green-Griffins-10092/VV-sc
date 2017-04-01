package org.firstinspires.ftc.griffins.Testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.griffins.AutoFunctions;
import org.firstinspires.ftc.griffins.RobotHardware;

/**
 * Created by David on 3/30/2017.
 */

@Autonomous
public class AutoAutoLoadingTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        RobotHardware hardware = new RobotHardware();
        hardware.initialize(hardwareMap);
        AutoFunctions autoFunctions = new AutoFunctions(hardware, this, RobotHardware.BeaconState.RED);
        hardware.registerLoaderColorSensor();

        waitForStart();

        autoFunctions.AutoLoadingTest();
    }
}
