package org.firstinspires.ftc.griffins.Testing;

import android.content.Context;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.internal.AppUtil;

import java.io.File;
import java.util.LinkedList;
import java.util.List;

/**
 * Created by David on 11/20/2016.
 */

@TeleOp(name = "Android Accelerometer Test", group = "test")
@Disabled
public class AndroidAccelerometerTest extends OpMode implements SensorEventListener {

    Sensor sensor;
    SensorManager sensorManager;
    List<Acceleration> log;  // timestamp, x axis, y axis, z axis
    boolean aPreviousState;

    @Override
    public void init() {
        sensorManager = (SensorManager) hardwareMap.appContext.getSystemService(Context.SENSOR_SERVICE);
        sensor = sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
        log = new LinkedList<>();
    }

    @Override
    public void start() {
        super.start();
        sensorManager.registerListener(this, sensor, SensorManager.SENSOR_DELAY_GAME);
    }

    @Override
    public void loop() {
        telemetry.addData("acceleration data", log.isEmpty() ? "no value" : log.get(log.size() - 1));
        if (gamepad1.a && !aPreviousState) {
            StringBuilder stringBuilder = new StringBuilder();
            for (Acceleration accel :
                    log) {
                stringBuilder.append(accel.acquisitionTime + ", " + accel.xAccel + ", " + accel.yAccel + ", " + accel.zAccel + "\n");
            }
            String filename = "AccelerometerDataAndroid.csv";
            File file = AppUtil.getInstance().getSettingsFile(filename);
            ReadWriteFile.writeFile(file, stringBuilder.toString());
            telemetry.log().add("saved to '%s'", filename);
        }
        aPreviousState = gamepad1.a;
    }

    @Override
    public void stop() {
        super.stop();
        sensorManager.unregisterListener(this);
    }

    @Override
    public void onSensorChanged(SensorEvent event) {
        //add to log  // TODO: 11/20/2016 check units
        log.add(new Acceleration(DistanceUnit.METER, event.values[0], event.values[1], event.values[2], event.timestamp));
    }

    @Override
    public void onAccuracyChanged(Sensor sensor, int accuracy) {
        //do nothing
    }
}
