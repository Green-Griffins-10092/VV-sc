package org.firstinspires.ftc.griffins;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by David on 12/7/2016.
 */
@Autonomous(name = "Cat Ball Auto 3", group = "Competition")
public class CatBallAuto3 extends LinearOpMode {
    private RobotHardware hardware;
    private AutoFunctions autoFunctions;

    @Override
    public void runOpMode() throws InterruptedException {
        hardware = new RobotHardware();
        hardware.initialize(hardwareMap);
        autoFunctions = new AutoFunctions(hardware, this);

        double shootingDistance = 50;
        double totalDriveDistance = 65;

        waitForStart();
        sleep(15000);

        autoFunctions.driveStraightPID(totalDriveDistance - shootingDistance, AutoFunctions.DriveStraightDirection.FORWARD);
        autoFunctions.shoot();
        hardware.getIntake().setPower(-1.0);
        autoFunctions.driveStraightPID(shootingDistance, AutoFunctions.DriveStraightDirection.FORWARD);

        hardware.getIntake().setPower(0.0);
    }
}
