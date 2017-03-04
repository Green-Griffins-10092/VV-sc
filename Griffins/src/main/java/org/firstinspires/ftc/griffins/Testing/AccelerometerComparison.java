package org.firstinspires.ftc.griffins.Testing;

import android.content.Context;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cCompassSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.List;

/**
 * Created by David on 11/20/2016.
 */

@TeleOp(name = "Accelerometer Comparison", group = "test")
@Disabled
public class AccelerometerComparison extends OpMode implements SensorEventListener {

    final Object lock = new Object();
    ModernRoboticsI2cCompassSensor accelerometer;
    BNO055IMU imu;
    Sensor sensor;
    SensorManager sensorManager;
    Acceleration androidData;

    @Override
    public void init() {
        accelerometer = hardwareMap.getAll(ModernRoboticsI2cCompassSensor.class).get(0);

        List<BNO055IMU> imus = hardwareMap.getAll(BNO055IMU.class);
        imu = imus.get(0);

        BNO055IMU.Parameters parameters;
        parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json";
        parameters.loggingEnabled = true;

        imu.initialize(parameters);

        sensorManager = (SensorManager) hardwareMap.appContext.getSystemService(Context.SENSOR_SERVICE);
        sensor = sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
    }

    @Override
    public void start() {
        super.start();
        sensorManager.registerListener(this, sensor, SensorManager.SENSOR_DELAY_GAME);
    }

    @Override
    public void loop() {
        synchronized (lock) {
            telemetry.addData("Adafruit", imu.getAcceleration());
            telemetry.addData("MR", accelerometer.getAcceleration());
            telemetry.addData("Android", androidData);
        }
    }

    @Override
    public void stop() {
        super.stop();
        sensorManager.unregisterListener(this);
    }

    @Override
    public void onSensorChanged(SensorEvent event) {
        synchronized (lock) {
            androidData = new Acceleration(DistanceUnit.METER, event.values[0], event.values[1], event.values[2], event.timestamp);
        }
    }

    @Override
    public void onAccuracyChanged(Sensor sensor, int accuracy) {

    }
}
