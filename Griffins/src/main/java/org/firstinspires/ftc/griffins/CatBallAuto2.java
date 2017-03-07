package org.firstinspires.ftc.griffins;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by David on 12/7/2016.
 */
@Autonomous(name = "Cat Ball Auto 2", group = "Competition")
@Disabled
public class CatBallAuto2 extends LinearOpMode {
    private RobotHardware hardware;
    private AutoFunctions autoFunctions;

    @Override
    public void runOpMode() throws InterruptedException {
        hardware = new RobotHardware();
        hardware.initialize(hardwareMap);
        autoFunctions = new AutoFunctions(hardware, this);

        double firstDriveDistance = 10;
        double totalDriveDistance = 50;

        waitForStart();
        sleep(15000);
        autoFunctions.driveStraightPID(firstDriveDistance, AutoFunctions.DriveStraightDirection.FORWARD);
        autoFunctions.shoot();
        hardware.getIntake().setPower(-1.0);
        autoFunctions.driveStraightPID(totalDriveDistance - firstDriveDistance, AutoFunctions.DriveStraightDirection.FORWARD);

        hardware.getIntake().setPower(0.0);
    }
}
