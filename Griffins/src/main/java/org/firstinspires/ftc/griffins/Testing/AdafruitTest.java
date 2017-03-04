package org.firstinspires.ftc.griffins.Testing;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.griffins.GriffinAccelerationIntegratorLowPass;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.internal.AppUtil;

import java.io.File;
import java.util.List;
import java.util.Locale;

/**
 * Created by David on 10/3/2016.
 * To test the new BNO055 sensor
 */

@TeleOp(name = "Adafruit IMU Test", group = "test")
@Disabled
public class AdafruitTest extends OpMode {

    BNO055IMU imu;

    Acceleration gravity;
    Orientation angles;
    Acceleration algorithmAcceleration;
    Acceleration linearAcceleration;
    Acceleration overallAcceleration;
    boolean pressedA = false;
    private boolean pressedB = false;
    private GriffinAccelerationIntegratorLowPass integrator;

    @Override
    public void init() {
        List<BNO055IMU> imus = hardwareMap.getAll(BNO055IMU.class);
        imu = imus.get(0);

        BNO055IMU.Parameters parameters;
        parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        integrator = new GriffinAccelerationIntegratorLowPass();
        parameters.accelerationIntegrationAlgorithm = integrator;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json";
        parameters.loggingEnabled = true;

        imu.initialize(parameters);

        composeTelemetry();
        telemetry.log().add("Press A to (re)start acceleration integration in loop");
        telemetry.log().add("remember to hold the sensor still when (re)starting integration");
        telemetry.log().add("Press B to write acceleration data");
    }

    @Override
    public void loop() {

        if (gamepad1.a && !pressedA) {
            imu.startAccelerationIntegration(new Position(), new Velocity(), 10);
        }

        if (gamepad1.b && !pressedB) {
            String filename = "AccelerometerDataAdafruit.csv";
            File file = AppUtil.getInstance().getSettingsFile(filename);
            ReadWriteFile.writeFile(file, integrator.getLog());
            telemetry.log().add("saved to '%s'", filename);
        }

        pressedA = gamepad1.a;
        pressedB = gamepad1.b;
    }

    void composeTelemetry() {

        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() {
            @Override
            public void run() {
                // Acquiring the angles is relatively expensive; we don't want
                // to do that in each of the three items that need that info, as that's
                // three times the necessary expense.
                angles = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
                gravity = imu.getGravity();
                algorithmAcceleration = imu.getAcceleration();
                linearAcceleration = imu.getLinearAcceleration();
                overallAcceleration = imu.getOverallAcceleration();
            }
        });

        telemetry.addLine()
                .addData("status", new Func<String>() {
                    @Override
                    public String value() {
                        return imu.getSystemStatus().toShortString();
                    }
                })
                .addData("calib", new Func<String>() {
                    @Override
                    public String value() {
                        return imu.getCalibrationStatus().toString();
                    }
                });

        telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override
                    public String value() {
                        return formatAngle(angles.angleUnit, angles.firstAngle);
                    }
                })
                .addData("roll", new Func<String>() {
                    @Override
                    public String value() {
                        return formatAngle(angles.angleUnit, angles.secondAngle);
                    }
                })
                .addData("pitch", new Func<String>() {
                    @Override
                    public String value() {
                        return formatAngle(angles.angleUnit, angles.thirdAngle);
                    }
                });

        telemetry.addLine()
                .addData("grvty", new Func<String>() {
                    @Override
                    public String value() {
                        return gravity.toString();
                    }
                })
                .addData("mag", new Func<String>() {
                    @Override
                    public String value() {
                        return String.format(Locale.getDefault(), "%.3f",
                                Math.sqrt(gravity.xAccel * gravity.xAccel
                                        + gravity.yAccel * gravity.yAccel
                                        + gravity.zAccel * gravity.zAccel));
                    }
                });

        telemetry.addLine()
                .addData("alg acl", new Func<String>() {
                    @Override
                    public String value() {
                        return algorithmAcceleration.toString();
                    }
                })
                .addData("mag", new Func<String>() {
                    @Override
                    public String value() {
                        return String.format(Locale.getDefault(), "%.3f",
                                Math.sqrt(algorithmAcceleration.xAccel * algorithmAcceleration.xAccel
                                        + algorithmAcceleration.yAccel * algorithmAcceleration.yAccel
                                        + algorithmAcceleration.zAccel * algorithmAcceleration.zAccel));
                    }
                });

        telemetry.addLine()
                .addData("ova acl", new Func<String>() {
                    @Override
                    public String value() {
                        return overallAcceleration.toString();
                    }
                })
                .addData("mag", new Func<String>() {
                    @Override
                    public String value() {
                        return String.format(Locale.getDefault(), "%.3f",
                                Math.sqrt(overallAcceleration.xAccel * overallAcceleration.xAccel
                                        + overallAcceleration.yAccel * overallAcceleration.yAccel
                                        + overallAcceleration.zAccel * overallAcceleration.zAccel));
                    }
                });

        telemetry.addLine()
                .addData("lin acl", new Func<String>() {
                    @Override
                    public String value() {
                        return linearAcceleration.toString();
                    }
                })
                .addData("mag", new Func<String>() {
                    @Override
                    public String value() {
                        return String.format(Locale.getDefault(), "%.3f",
                                Math.sqrt(linearAcceleration.xAccel * linearAcceleration.xAccel
                                        + linearAcceleration.yAccel * linearAcceleration.yAccel
                                        + linearAcceleration.zAccel * linearAcceleration.zAccel));
                    }
                });

        telemetry.addLine()
                .addData("pos", new Func<String>() {
                    @Override
                    public String value() {
                        Position pos = imu.getPosition();
                        return String.format(Locale.getDefault(), "x:%.2f, y%.2f, z:%.2f", pos.x, pos.y, pos.z);
                    }
                }).addData("vel", new Func<String>() {
            @Override
            public String value() {
                Velocity vel = imu.getVelocity();
                return String.format(Locale.getDefault(), "x:%.2f, y%.2f, z:%.2f", vel.xVeloc, vel.yVeloc, vel.zVeloc);
            }
        });
    }

    //----------------------------------------------------------------------------------------------
    // Formatting
    //----------------------------------------------------------------------------------------------

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees) {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
}
