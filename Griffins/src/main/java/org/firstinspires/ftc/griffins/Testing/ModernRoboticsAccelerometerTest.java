package org.firstinspires.ftc.griffins.Testing;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cCompassSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.internal.AppUtil;

import java.io.File;
import java.util.LinkedList;
import java.util.List;

/**
 * Created by David on 11/20/2016.
 */

@TeleOp(name = "MR acceleration test", group = "test")
@Disabled
public class ModernRoboticsAccelerometerTest extends OpMode {

    ModernRoboticsI2cCompassSensor accelerometer;
    List<Acceleration> log;
    boolean aPreviousState;

    @Override
    public void init() {
        accelerometer = hardwareMap.getAll(ModernRoboticsI2cCompassSensor.class).get(0);
        log = new LinkedList<>();
    }

    @Override
    public void loop() {
        log.add(accelerometer.getAcceleration());
        telemetry.addData("acceleration data", accelerometer.getAcceleration());
        telemetry.addData("compass direction", accelerometer.getDirection());
        telemetry.addData("magnetic flux", accelerometer.getMagneticFlux());

        if (gamepad1.a && !aPreviousState) {
            StringBuilder stringBuilder = new StringBuilder();
            for (Acceleration accel :
                    log) {
                stringBuilder.append(accel.acquisitionTime + ", " + accel.xAccel + ", " + accel.yAccel + ", " + accel.zAccel + "\n");
            }
            String filename = "AccelerometerDataMR.csv";
            File file = AppUtil.getInstance().getSettingsFile(filename);
            ReadWriteFile.writeFile(file, stringBuilder.toString());
            telemetry.log().add("saved to '%s'", filename);
        }
        aPreviousState = gamepad1.a;
    }
}
