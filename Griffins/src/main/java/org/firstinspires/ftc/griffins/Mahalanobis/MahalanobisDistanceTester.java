package org.firstinspires.ftc.griffins.Mahalanobis;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.griffins.Mahalanobis.ColorSensorDistance.ColorLabels;
import org.firstinspires.ftc.griffins.Mahalanobis.ColorSensorDistance.ColorTrainingData;
import org.firstinspires.ftc.griffins.RobotHardware;

/**
 * Created by David on 2/27/2017.
 */

public class MahalanobisDistanceTester extends OpMode {

    ColorSensorDistance classifier;
    RobotHardware hardware = new RobotHardware();
    private ColorLabels alliance = ColorLabels.RED;

    @Override
    public void init() {
        hardware.initialize(hardwareMap);
        classifier = ColorSensorDistance.getColorSensorDistanceFromData((ColorTrainingData[]) DataCollector.data.toArray());
        telemetry.log().add("Classifier constructed");
    }

    @Override
    public void start() {
        super.start();
        if (gamepad1.b) {
            alliance = ColorLabels.RED;
        } else if (gamepad1.x) {
            alliance = ColorLabels.BLUE;
        }

        telemetry.log().add("alliance is " + alliance);
    }

    @Override
    public void loop() {
        ColorSensorDistance.ColorData colorData = new ColorSensorDistance.ColorData(hardware.getLoaderColorSensor());
        ColorLabels label = classifier.findColor(colorData);

        hardware.setDrivePower(-gamepad1.left_stick_y, -gamepad1.right_stick_y);
        hardware.getIntake().setPower(gamepad1.left_bumper ? -1 : gamepad1.left_trigger);

        if (label == ColorLabels.UNDEFINED) {
            hardware.setLoaderPower(gamepad1.right_bumper ? -1 : gamepad1.right_trigger);
        } else if (label == alliance) {
            hardware.setLoaderPower(1);
        } else {
            hardware.setLoaderPower(-1);
            hardware.getIntake().setPower(-1);
        }

        telemetry.addData("particle color", label);
        telemetry.addData("color data (a, r, g, b)", colorData);
    }
}
