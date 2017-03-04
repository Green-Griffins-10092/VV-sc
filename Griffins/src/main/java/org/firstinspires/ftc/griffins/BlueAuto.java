package org.firstinspires.ftc.griffins;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.griffins.MenuPort.FtcMenu;
import org.firstinspires.ftc.griffins.MenuPort.HalDashboard;
import org.firstinspires.ftc.griffins.RobotHardware.BeaconState;
import org.firstinspires.ftc.griffins.Testing.WallApproachTest;

/**
 * Created by David on 12/7/2016.
 */
@Autonomous(name = "Blue Beacon Auto", group = "Competition")
public class BlueAuto extends LinearOpMode implements FtcMenu.MenuButtonsAndDashboard {
    HalDashboard halDashboard;
    private RobotHardware hardware;
    private AutoFunctions autoFunctions;

    public BlueAuto() {
        halDashboard = HalDashboard.getInstance(telemetry);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        hardware = new RobotHardware();
        hardware.initialize(hardwareMap);
        autoFunctions = new AutoFunctions(hardware, this);

        Alliance alliance = Alliance.BLUE_ALLIANCE;
        telemetry.log().add("Alliance is " + alliance);

        waitForStart();

        while (opModeIsActive() && hardware.getTurretGyro().isCalibrating()) ;

        //shoot two particles
        autoFunctions.shoot();
        telemetry.log().add("Finished Shooting");
        telemetry.update();

        //drive straight a little to get off wall in order to turn
        autoFunctions.driveStraightPID(25, AutoFunctions.DriveStraightDirection.FORWARD, 3);
        telemetry.log().add("Off the Wall");
        telemetry.update();

        //turn so facing toward beacon
        autoFunctions.twoWheelTurnPID(45, AutoFunctions.TurnDirection.RIGHT, 8);
        telemetry.log().add("Turned towards beacon");
        telemetry.update();

        double angle = autoFunctions.getZAngle();

        //drive toward beacon wall
        autoFunctions.driveStraightPID(88, AutoFunctions.DriveStraightDirection.FORWARD, 3, true);
        telemetry.log().add("Arrived at beacon wall");
        telemetry.update();

        autoFunctions.driveStraightPID(3, AutoFunctions.DriveStraightDirection.BACKWARD, 1);

        angle -= autoFunctions.getZAngle();

        //"parallel parking"
        WallApproachTest.blueWallApproach(hardware, autoFunctions, this, (int) angle);

        if (hardware.findParticleColor() == BeaconState.RED) {
            hardware.setLoaderPower(-1);
        }

        autoFunctions.scanForBeacon(AutoFunctions.DriveStraightDirection.FORWARD);

        hardware.setLoaderPower(0);

        hardware.setDrivePower(-0.2, -0.1);

        sleep(200);

        hardware.setDrivePower(-0.3, 0.3);

        sleep(200);
        hardware.stopDrive();

        autoFunctions.pushBeacon(BeaconState.BLUE);

        BeaconState state = hardware.findBeaconState();
        if (state != BeaconState.BLUE_BLUE && !state.containsUndefined()) {
            if (getRuntime() >= 28) {
                hardware.pushButtonFullExtension(state, BeaconState.BLUE);
            } else if (getRuntime() >= 20) {
                sleep((long) ((28 - getRuntime()) * 1000));
                hardware.pushButtonFullExtension(state, BeaconState.BLUE);
            } else {
                sleep(6000);
                hardware.pushButtonFullExtension(state, BeaconState.BLUE);
            }

            sleep(2000);
            hardware.pushButton(BeaconState.UNDEFINED, BeaconState.BLUE);
        }

        //autoFunctions.twoWheelTurnPID(45, AutoFunctions.TurnDirection.LEFT, 0.5, true); //timer out
        autoFunctions.wallPIDDrive(43, AutoFunctions.DriveStraightDirection.BACKWARD, 2);

        autoFunctions.scanForBeacon(AutoFunctions.DriveStraightDirection.BACKWARD);

        hardware.setDrivePower(-0.2, -0.1);

        sleep(400);
        hardware.stopDrive();

        autoFunctions.pushBeacon(BeaconState.BLUE);

        state = hardware.findBeaconState();
        if (state != BeaconState.BLUE_BLUE && !state.containsUndefined()) {
            if (getRuntime() >= 28) {
                hardware.pushButtonFullExtension(state, BeaconState.BLUE);
            } else if (getRuntime() >= 20) {
                sleep((long) ((28 - getRuntime()) * 1000));
                hardware.pushButtonFullExtension(state, BeaconState.BLUE);
            } else {
                sleep(6000);
                hardware.pushButtonFullExtension(state, BeaconState.BLUE);
            }

            sleep(2000);
            hardware.pushButton(BeaconState.UNDEFINED, BeaconState.BLUE);
        }

        hardware.setDrivePower(0.2, 0.3);
        sleep(800);
        hardware.setDrivePower(0, -0.6);
        sleep(800);
        hardware.setDrivePower(-0.5, -0.3);
        sleep(2000);
        hardware.stopDrive();

        /*//drive back to first beacon
        autoFunctions.driveStraight((long)(34/hardware.INCHES_PER_ENCODER_COUNT), AutoFunctions.DriveStraightDirection.FORWARD, .5);
        telemetry.log().add("Arrived at first beacon");
        telemetry.update();

        //button pusher works its magic
        if(opModeIsActive()){
            hardware.pushButton(hardware.findBeaconState(), RobotHardware.BeaconState.BLUE_BLUE);
        }
        telemetry.log().add("Pushed first button");
        telemetry.update();

        //drive straight until in front of second beacon
        autoFunctions.driveStraight((long)(50/hardware.INCHES_PER_ENCODER_COUNT), AutoFunctions.DriveStraightDirection.BACKWARD, .5);
        telemetry.log().add("Arrived at second beacon");
        telemetry.update();

        //button pusher works its magic pt 2
        if(opModeIsActive()){
            hardware.pushButton(hardware.findBeaconState(), RobotHardware.BeaconState.BLUE_BLUE);
        }
        telemetry.log().add("Pushed second button");
        telemetry.update();*/
    }

    @Override
    public boolean isMenuUpButton() {
        return gamepad1.dpad_up;
    }

    @Override
    public boolean isMenuDownButton() {
        return gamepad1.dpad_down;
    }

    @Override
    public boolean isMenuEnterButton() {
        return gamepad1.a;
    }

    @Override
    public boolean isMenuBackButton() {
        return gamepad1.b;
    }

    @Override
    public HalDashboard getHalDashboard() {
        halDashboard.resetTelemetryForHalDashboard();
        return halDashboard;
    }

    private enum Alliance {
        RED_ALLIANCE,
        BLUE_ALLIANCE
    }   //enum Alliance
}