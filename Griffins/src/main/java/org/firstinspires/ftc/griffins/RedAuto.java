package org.firstinspires.ftc.griffins;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.griffins.RobotHardware.BeaconState;
import org.firstinspires.ftc.griffins.Testing.WallApproachTest;

/**
 * Created by David on 12/7/2016.
 */
@Autonomous(name = "Red Beacon Auto", group = "Competition")
public class RedAuto extends LinearOpMode {
    private RobotHardware hardware;
    private AutoFunctions autoFunctions;

    @Override
    public void runOpMode() throws InterruptedException {
        hardware = new RobotHardware();
        hardware.initialize(hardwareMap);
        autoFunctions = new AutoFunctions(hardware, this);

        Alliance alliance = Alliance.RED_ALLIANCE;
        telemetry.log().add("Alliance is " + alliance);

        waitForStart();

        while (opModeIsActive() && hardware.getTurretGyro().isCalibrating()) ;

        //shoot two particles
        autoFunctions.shoot();
        telemetry.log().add("Finished Shooting");
        telemetry.update();

        //drive straight a little to get off wall in order to turn
        autoFunctions.driveStraightPID(20, AutoFunctions.DriveStraightDirection.FORWARD, 3);
        telemetry.log().add("Off the Wall");
        telemetry.update();

        //turn so facing toward beacon
        autoFunctions.twoWheelTurnPID(119, AutoFunctions.TurnDirection.RIGHT, 8);
        telemetry.log().add("Turned towards beacon");
        telemetry.update();

        double angle = autoFunctions.getZAngle();

        //drive toward beacon wall
        autoFunctions.driveStraightPID(59, AutoFunctions.DriveStraightDirection.BACKWARD, 3);
        telemetry.log().add("Arrived at beacon wall");
        telemetry.update();

        autoFunctions.driveStraightPID(2, AutoFunctions.DriveStraightDirection.FORWARD, .5, true);

        angle -= autoFunctions.getZAngle();

        WallApproachTest.redWallApproach(hardware, autoFunctions, this, (int) angle);

        autoFunctions.scanForBeacon(AutoFunctions.DriveStraightDirection.BACKWARD);

        hardware.setDrivePower(-0.2, -0.1);

        sleep(200);
        hardware.stopDrive();

        autoFunctions.pushBeacon(BeaconState.RED);

        BeaconState state = hardware.findBeaconState();
        if (state != BeaconState.RED_RED && !state.containsUndefined()) {
            if (getRuntime() >= 28) {
                hardware.pushButtonFullExtension(state, BeaconState.RED);
            } else if (getRuntime() >= 20) {
                sleep((long) ((28 - getRuntime()) * 1000));
                hardware.pushButtonFullExtension(state, BeaconState.RED);
            } else {
                sleep(6000);
                hardware.pushButtonFullExtension(state, BeaconState.RED);
            }

            sleep(2000);
            hardware.pushButton(BeaconState.UNDEFINED, BeaconState.RED);
            sleep(1000);
        }

        //autoFunctions.twoWheelTurnPID(90, AutoFunctions.TurnDirection.RIGHT, 0.3); //timer out
        autoFunctions.wallPIDDrive(43, AutoFunctions.DriveStraightDirection.FORWARD, 2);

        autoFunctions.scanForBeacon(AutoFunctions.DriveStraightDirection.FORWARD);
        hardware.setDrivePower(-0.2, -0.1);

        sleep(200);
        hardware.setDrivePower(-0.3, 0.3);
        sleep(300);
        hardware.stopDrive();

        autoFunctions.pushBeacon(BeaconState.RED);

        state = hardware.findBeaconState();
        if (state != BeaconState.RED_RED && !state.containsUndefined()) {
            if (getRuntime() >= 28) {
                hardware.pushButtonFullExtension(state, BeaconState.RED);
            } else if (getRuntime() >= 20) {
                sleep((long) ((28 - getRuntime()) * 1000));
                hardware.pushButtonFullExtension(state, BeaconState.RED);
            } else {
                sleep(6000);
                hardware.pushButtonFullExtension(state, BeaconState.RED);
            }

            sleep(2000);
            hardware.pushButton(BeaconState.UNDEFINED, BeaconState.RED);
            sleep(1000);
        }

        hardware.setDrivePower(-0.2, -0.3);
        sleep(800);
        hardware.setDrivePower(0, 0.6);
        sleep(800);
        hardware.setDrivePower(0.5, 0.3);
        sleep(2000);
        hardware.stopDrive();

    }

    private enum Alliance {
        RED_ALLIANCE,
        BLUE_ALLIANCE
    }   //enum Alliance
}