package org.firstinspires.ftc.griffins.Mahalanobis;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.griffins.Mahalanobis.ColorSensorDistance.ColorLabels;
import org.firstinspires.ftc.griffins.Mahalanobis.ColorSensorDistance.ColorTrainingData;
import org.firstinspires.ftc.griffins.RobotHardware;

import java.util.ArrayList;

/**
 * Created by David on 2/27/2017.
 */

public class DataCollector extends OpMode {
    public static ArrayList<ColorTrainingData> data = new ArrayList<>();

    private RobotHardware hardware = new RobotHardware();
    private boolean isPressed = false;


    @Override
    public void init() {
        hardware.initialize(hardwareMap);
    }

    @Override
    public void loop() {

        hardware.setDrivePower(-gamepad1.left_stick_y, -gamepad1.right_stick_y);
        hardware.getIntake().setPower(gamepad1.left_bumper ? -1 : gamepad1.left_trigger);
        hardware.setLoaderPower(gamepad1.right_bumper ? -1 : gamepad1.right_trigger);

        if (!isPressed) {
            if (gamepad1.a) { //undefined
                addTrainingData(ColorLabels.UNDEFINED);
            } else if (gamepad1.b) { //red
                addTrainingData(ColorLabels.RED);
            } else if (gamepad1.x) { //blue
                addTrainingData(ColorLabels.BLUE);
            }
        }

        isPressed = gamepad1.a || gamepad1.b || gamepad1.x;

    }

    private void addTrainingData(ColorLabels label) {
        ColorSensor theSensor = hardware.getLoaderColorSensor();
        ColorTrainingData trainingData = new ColorTrainingData(theSensor, label);
        data.add(trainingData);
        telemetry.log().add("data logged as " + label + ", with data " + trainingData);
    }
}
