package org.firstinspires.ftc.griffins;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.griffins.MenuPort.HalDashboard;

import static org.firstinspires.ftc.griffins.RobotHardware.ENCODER_COUNTS_PER_INCH;

/**
 * Created by David on 12/7/2016.
 */
@Autonomous(name = "Cat Ball Auto 2", group = "Competition")
@Disabled
public class CatBallAuto2 extends LinearOpMode {
    public static final double countsPerRobotRotation = ENCODER_COUNTS_PER_INCH * Math.PI * 14.5625;
    HalDashboard halDashboard;
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
