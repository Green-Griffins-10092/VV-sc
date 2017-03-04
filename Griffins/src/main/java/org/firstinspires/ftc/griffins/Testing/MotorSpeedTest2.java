package org.firstinspires.ftc.griffins.Testing;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsUsbDcMotorController;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.griffins.MenuPort.FtcChoiceMenu;
import org.firstinspires.ftc.griffins.MenuPort.FtcMenu;
import org.firstinspires.ftc.griffins.MenuPort.FtcValueMenu;
import org.firstinspires.ftc.griffins.MenuPort.HalDashboard;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Iterator;

/**
 * Created by David on 10/22/2016.
 */
@TeleOp(group = "test")
@Disabled
public class MotorSpeedTest2 extends LinearOpMode implements FtcMenu.MenuButtonsAndDashboard {

    public static final int DEFAULT_RATE_TRACKING_WINDOW = 2000; // In milliseconds

    @Override
    public void runOpMode() throws InterruptedException {
        Iterator<DcMotor> motorIterator = hardwareMap.dcMotor.iterator();
        HalDashboard dashboard = HalDashboard.getInstance(telemetry);
        final DcMotor motor = motorIterator.next();
        DcMotor motor2 = null;
        if (motorIterator.hasNext()) {
            motor2 = motorIterator.next();
        }
        ElapsedTime time = new ElapsedTime();
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor.setDirection(DcMotorSimple.Direction.FORWARD);
        if (motor2 != null) {
            motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            motor2.setDirection(DcMotorSimple.Direction.FORWARD);
        }

        telemetry.log().setCapacity(20);

        FtcValueMenu rateTrackingWindowMenu = new FtcValueMenu("Rate Tracking Window:", null, this, 500, 10_000, 500, DEFAULT_RATE_TRACKING_WINDOW, "%f ms");
        FtcChoiceMenu motorModeMenu = new FtcChoiceMenu("Motor Mode>", rateTrackingWindowMenu, this);
        FtcValueMenu motorSpeedMenu = new FtcValueMenu("Motor Speed:", motorModeMenu, this, 0, 1, 0.05, .5, "%f");
        motorModeMenu.addChoice("Run Without Encoders", DcMotor.RunMode.RUN_WITHOUT_ENCODER, motorSpeedMenu);
        motorModeMenu.addChoice("Run With Encoders", DcMotor.RunMode.RUN_USING_ENCODER, motorSpeedMenu);
        rateTrackingWindowMenu.setChildMenu(motorModeMenu);

        telemetry.log().add("motor info: Connection Info %s, PID Constants %s, Gear Ratio %s", motor.getConnectionInfo(),
                ((ModernRoboticsUsbDcMotorController) motor.getController()).getDifferentialControlLoopCoefficients(motor.getPortNumber()),
                ((ModernRoboticsUsbDcMotorController) motor.getController()).getGearRatio(motor.getPortNumber()));

        if (motor2 != null) {
            telemetry.log().add("motor 2 info: Connection Info %s, PID Constants %s, Gear Ratio %s", motor2.getConnectionInfo(),
                    ((ModernRoboticsUsbDcMotorController) motor2.getController()).getDifferentialControlLoopCoefficients(motor2.getPortNumber()),
                    ((ModernRoboticsUsbDcMotorController) motor2.getController()).getGearRatio(motor2.getPortNumber()));
        }

        telemetry.log().add("Procedure: Use the menus to determine motor parameters, using the d_pad. Then press a to start tracking");

        waitForStart();

        while (opModeIsActive()) {
            double speed;
            int encoderPosition1;
            DcMotor.RunMode mode;
            int encoderPosition2 = 0;
            int sampleWindow = DEFAULT_RATE_TRACKING_WINDOW;

            dashboard.resetTelemetryForHalDashboard();
            FtcMenu.walkMenuTree(motorModeMenu, this, false);
            dashboard.resetTelemetryForOpMode();

            telemetry.addData("Voltage", new Func<Double>() {
                @Override
                public Double value() {
                    return ((ModernRoboticsUsbDcMotorController) motor.getController()).getVoltage();
                }
            });

            mode = (DcMotor.RunMode) motorModeMenu.getCurrentChoiceObject();
            speed = motorSpeedMenu.getCurrentValue();
            sampleWindow = (int) rateTrackingWindowMenu.getCurrentValue();

            motor.setMode(mode);
            if (motor2 != null) {
                motor2.setMode(mode);
            }

            motor.setPower(speed);
            if (motor2 != null) {
                motor2.setPower(speed);
            }

            while (!gamepad1.a && opModeIsActive()) ;

            encoderPosition1 = motor.getCurrentPosition();
            if (motor2 != null) {
                encoderPosition2 = motor2.getCurrentPosition();
            }
            time.reset();
            Telemetry.Item rate = telemetry.addData("motor rate (counts/sec)", new Func<Double>() {
                ElapsedTime timer = new ElapsedTime();
                long previousPosition = 0;

                @Override
                public Double value() {
                    double value = (motor.getCurrentPosition() - previousPosition) / timer.seconds();
                    previousPosition = motor.getCurrentPosition();
                    timer.reset();
                    return value;
                }
            });

            sleep(DEFAULT_RATE_TRACKING_WINDOW);

            telemetry.removeItem(rate);

            double rate1 = (motor.getCurrentPosition() - encoderPosition1) / time.seconds();
            double rate2 = 0;
            if (motor2 != null) {
                rate2 = (motor2.getCurrentPosition() - encoderPosition2) / time.seconds();
            }

            telemetry.log().add("motor power, mode, sample window: %.2f, %s, %.1f", motor.getPower(), motor.getMode(), sampleWindow / 1000.0);
            if (motor2 != null) {
                telemetry.log().add("encoder counts per second: %.2f, %.2f", rate1, rate2);
            } else {
                telemetry.log().add("encoder counts per second: %.2f, %.2f", rate1);
                telemetry.log().add("encoder count rate difference: %.5f", Math.abs(rate1 - rate2));
            }
            telemetry.log().add("------------");
            telemetry.update();

            motor.setPower(0);
            if (motor2 != null) {
                motor2.setPower(0);
            }
        }

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
        return gamepad1.dpad_right;
    }

    @Override
    public boolean isMenuBackButton() {
        return gamepad1.dpad_left;
    }

    @Override
    public HalDashboard getHalDashboard() {
        return HalDashboard.getInstance(telemetry);
    }
}
