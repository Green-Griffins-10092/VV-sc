package org.firstinspires.ftc.griffins.Testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.griffins.RobotHardware;

import static org.firstinspires.ftc.griffins.RobotHardware.BUTTON_PUSHER_CENTER_POSITION;
import static org.firstinspires.ftc.griffins.RobotHardware.BUTTON_PUSHER_LEFT_FULL_EXTENSION;

/**
 * Created by David on 12/15/2016.
 */

@Autonomous(name = "Beacon Pushing Test", group = "test")
@Disabled
public class BeaconTest extends LinearOpMode {
    public static final double BEACON_SERVO_INCREMENT = (BUTTON_PUSHER_LEFT_FULL_EXTENSION - BUTTON_PUSHER_CENTER_POSITION) / 15;
    private RobotHardware.BeaconState alliance = RobotHardware.BeaconState.RED_RED;

    @Override
    public void runOpMode() throws InterruptedException {
        RobotHardware hardware = new RobotHardware();
        hardware.initialize(hardwareMap);
        hardware.registerBeaconColorSensors();

        waitForStart();

        RobotHardware.BeaconState beaconState = hardware.findBeaconState();
        /*if (beaconState.containsUndefined()) {
            telemetry.log().add("Beacon state undefined, attempting active beacon state finder");
            sleep(1000);
            beaconState = activeBeaconStateFinder();
        }*/
        telemetry.addData("Beacon State", beaconState);
        telemetry.update();
        sleep(1000);
        hardware.pushButton(beaconState, RobotHardware.BeaconState.RED);
        sleep(3000);
        hardware.pushButton(RobotHardware.BeaconState.UNDEFINED, RobotHardware.BeaconState.RED);
        sleep(2000);
    }

    /*//blocking method
    RobotHardware.BeaconState activeBeaconStateFinder() {
        RobotHardware.BeaconState colorState = RobotHardware.BeaconState.UNDEFINED;
        RobotHardware.BeaconState leftColorState = RobotHardware.BeaconState.UNDEFINED;
        RobotHardware.BeaconState rightColorState = RobotHardware.BeaconState.UNDEFINED;
        double beaconServoPosition = BUTTON_PUSHER_CENTER_POSITION;

        for (int i = 0; i < 3; i++) {
            colorState = hardware.findBeaconState();
            if (colorState != RobotHardware.BeaconState.UNDEFINED)
                return colorState;
        }

        while (opModeIsActive() && leftColorState == RobotHardware.BeaconState.UNDEFINED) {
            if (beaconServoPosition < BUTTON_PUSHER_LEFT_FULL_EXTENSION) {
                return RobotHardware.BeaconState.UNDEFINED;
            }

            beaconServoPosition += BEACON_SERVO_INCREMENT;
            buttonPusherServo.setPosition(beaconServoPosition);
            sleep(250);
            leftColorState = findColorSensorState(leftButtonPusherColorSensor);
        }

        beaconServoPosition = BUTTON_PUSHER_CENTER_POSITION - (beaconServoPosition - BUTTON_PUSHER_CENTER_POSITION);
        buttonPusherServo.setPosition(beaconServoPosition);
        sleep(1000);
        rightColorState = findColorSensorState(rightButtonPusherColorSensor);

        colorState = mergeBeaconSideStates(leftColorState, rightColorState);

        return colorState;
    }*/
}
