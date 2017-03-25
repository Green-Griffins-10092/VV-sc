package org.firstinspires.ftc.griffins.Testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.griffins.AutoFunctions;
import org.firstinspires.ftc.griffins.RobotHardware;

/**
 * Created by David on 1/14/2017.
 */

@Autonomous(name = "Wall Approach Test", group = "test")
@Disabled
public class WallApproachTest extends LinearOpMode {
    public static void redWallApproach(RobotHardware hardware, AutoFunctions autoFunctions, LinearOpMode opMode, int angleDifference) {
        int gyroHeading = hardware.getTurretGyro().getIntegratedZValue();
        autoFunctions.twoWheelTurnPID(22 - angleDifference, AutoFunctions.TurnDirection.RIGHT, 3);

        hardware.registerBeaconColorSensors();
        hardware.registerLoaderColorSensor();

        autoFunctions.driveStraightPID(28, AutoFunctions.DriveStraightDirection.FORWARD, 1.5, true);

        hardware.getIntake().setPower(0);

        //autoFunctions.twoWheelTurnPID(20, AutoFunctions.TurnDirection.LEFT, .3, true); //timer out

        hardware.stopDrive();
    }

    public static void blueWallApproach(RobotHardware hardware, AutoFunctions autoFunctions, LinearOpMode opMode, int angleDifference) {
        int gyroHeading = hardware.getTurretGyro().getIntegratedZValue();
        autoFunctions.twoWheelTurnPID(22 + angleDifference, AutoFunctions.TurnDirection.LEFT, 3);

        hardware.registerBeaconColorSensors();
        hardware.registerLoaderColorSensor();

        autoFunctions.driveStraightPID(28, AutoFunctions.DriveStraightDirection.FORWARD, 1.5, true);

        hardware.getIntake().setPower(0);

        //autoFunctions.twoWheelTurnPID(20, AutoFunctions.TurnDirection.RIGHT, .3, true); //timer out

        hardware.stopDrive();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        final RobotHardware hardware = new RobotHardware();
        hardware.initialize(hardwareMap);
        AutoFunctions autoFunctions = new AutoFunctions(hardware, this);

        waitForStart();

        while (opModeIsActive() && hardware.getTurretGyro().isCalibrating()) ;

        blueWallApproach(hardware, autoFunctions, this, 0);

        sleep(500);

        hardware.setDrivePower(-.35, -.3);

        sleep(1000);

        hardware.stopDrive();
    }
}
