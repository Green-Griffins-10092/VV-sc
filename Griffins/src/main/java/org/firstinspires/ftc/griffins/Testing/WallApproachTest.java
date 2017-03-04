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
        autoFunctions.twoWheelTurnPID(38 - angleDifference, AutoFunctions.TurnDirection.RIGHT, 2.5);

        hardware.registerBeaconColorSensors();

        autoFunctions.driveStraightPID(25, AutoFunctions.DriveStraightDirection.BACKWARD, 1.5, true);

        //autoFunctions.twoWheelTurnPID(5, AutoFunctions.TurnDirection.RIGHT, 2, true);

        //autoFunctions.driveStraightPID(15, AutoFunctions.DriveStraightDirection.BACKWARD, 2, true);

        /*autoFunctions.twoWheelTurnPID(20, AutoFunctions.TurnDirection.LEFT, .5, true); //timer out

        autoFunctions.driveStraightPID(25, AutoFunctions.DriveStraightDirection.FORWARD, 1.5, true);*/

        /*hardware.setDrivePower(-.3, -.35);

        LinearOpModeTimeOutFunc timer = new LinearOpModeTimeOutFunc(opMode, 1);
        while (timer.value() && Math.abs(hardware.getTurretGyro().getIntegratedZValue() - gyroHeading + 70) > 1) {
            opMode.telemetry.addData("Gyro Heading", hardware.getTurretGyro().getIntegratedZValue());
            opMode.telemetry.addData("Gyro Target Error", hardware.getTurretGyro().getIntegratedZValue() - gyroHeading + 45);
            opMode.telemetry.update();
        }*/

        hardware.stopDrive();
    }

    public static void blueWallApproach(RobotHardware hardware, AutoFunctions autoFunctions, LinearOpMode opMode, int angleDifference) {
        int gyroHeading = hardware.getTurretGyro().getIntegratedZValue();
        autoFunctions.twoWheelTurnPID(30 + angleDifference, AutoFunctions.TurnDirection.LEFT, 3);

        hardware.registerBeaconColorSensors();
        hardware.registerLoaderColorSensor();

        autoFunctions.driveStraightPID(20, AutoFunctions.DriveStraightDirection.FORWARD, 1.5, true);

        hardware.getIntake().setPower(0);

        /*autoFunctions.twoWheelTurnPID(10, AutoFunctions.TurnDirection.RIGHT, 2);

        autoFunctions.driveStraightPID(16, AutoFunctions.DriveStraightDirection.FORWARD, 2);
*/
        autoFunctions.twoWheelTurnPID(20, AutoFunctions.TurnDirection.RIGHT, .3, true); //timer out

        //autoFunctions.driveStraightPID(10, AutoFunctions.DriveStraightDirection.FORWARD, 1.5);

        /*hardware.setDrivePower(.3, .35);

        LinearOpModeTimeOutFunc timer = new LinearOpModeTimeOutFunc(opMode, 2);
        while (timer.value() && Math.abs(hardware.getTurretGyro().getIntegratedZValue() - gyroHeading - 45) > 1) {
            opMode.telemetry.addData("Gyro Heading", hardware.getTurretGyro().getIntegratedZValue());
            opMode.telemetry.addData("Gyro Target Error", hardware.getTurretGyro().getIntegratedZValue() - gyroHeading + 45);
            opMode.telemetry.update();
        }*/

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
