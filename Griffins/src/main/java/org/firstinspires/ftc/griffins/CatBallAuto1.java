package org.firstinspires.ftc.griffins;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.griffins.MenuPort.FtcChoiceMenu;
import org.firstinspires.ftc.griffins.MenuPort.FtcMenu;
import org.firstinspires.ftc.griffins.MenuPort.FtcValueMenu;
import org.firstinspires.ftc.griffins.MenuPort.HalDashboard;

import static org.firstinspires.ftc.griffins.RobotHardware.ENCODER_COUNTS_PER_INCH;

/**
 * Created by David on 12/7/2016.
 */
@Autonomous(name = "Cat Ball Auto 1", group = "Competition")
public class CatBallAuto1 extends LinearOpMode implements FtcMenu.MenuButtonsAndDashboard {
    public static final double countsPerRobotRotation = ENCODER_COUNTS_PER_INCH * Math.PI * 14.5625;
    HalDashboard halDashboard;
    private RobotHardware hardware;
    private AutoFunctions autoFunctions;

    @Override
    public void runOpMode() throws InterruptedException {
        hardware = new RobotHardware();
        hardware.initialize(hardwareMap);
        autoFunctions = new AutoFunctions(hardware, this);

        FtcValueMenu firstDriveDistanceMenu = new FtcValueMenu("First Drive Distance", null, this, 0, 100, 1, 0, "%.0f in");
        FtcValueMenu secondDriveDistanceMenu = new FtcValueMenu("Second Drive Distance", firstDriveDistanceMenu, this,
                0, 100, 1, 48, "%.0f in");
        firstDriveDistanceMenu.setChildMenu(secondDriveDistanceMenu);
        FtcChoiceMenu allianceMenu = new FtcChoiceMenu("Alliance Menu: ", null, this);
        allianceMenu.addChoice("Red", Alliance.RED_ALLIANCE);
        allianceMenu.addChoice("Blue", Alliance.BLUE_ALLIANCE);

        FtcMenu.walkMenuTree(allianceMenu, this, true);
        halDashboard.resetTelemetryForOpMode();

        Alliance alliance = (Alliance) allianceMenu.getCurrentChoiceObject();
        double firstDriveDistance = 48;
        double secondDriveDistance = 48;
        /*telemetry.log().add("First Drive Distance is %.0f in", firstDriveDistance);
        telemetry.log().add("Second Drive Distance is %.0f in", secondDriveDistance);*/
        telemetry.log().add("Alliance is " + alliance);

        waitForStart();
        sleep(15000);
        //autoFunctions.driveStraightSimple((int) (firstDriveDistance * ENCODER_COUNTS_PER_INCH), AutoFunctions.DriveStraightDirection.FORWARD, .5);
        autoFunctions.shoot();
        hardware.getIntake().setPower(1.0);
        sleep(500);
        hardware.getIntake().setPower(-1.0);
        autoFunctions.driveStraightPID(50, AutoFunctions.DriveStraightDirection.FORWARD);

        if (alliance == Alliance.BLUE_ALLIANCE) {
            autoFunctions.twoWheelTurnSimple((int) (countsPerRobotRotation / 8), AutoFunctions.TurnDirection.RIGHT, 1);
            sleep(1000);
            autoFunctions.twoWheelTurnSimple((int) (countsPerRobotRotation / 8), AutoFunctions.TurnDirection.LEFT, 0.5);
        } else {
            autoFunctions.twoWheelTurnSimple((int) (countsPerRobotRotation / 8), AutoFunctions.TurnDirection.LEFT, 1);
            sleep(1000);
            autoFunctions.twoWheelTurnSimple((int) (countsPerRobotRotation / 8), AutoFunctions.TurnDirection.RIGHT, 0.5);
        }
        hardware.getIntake().setPower(0.0);
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
        if (halDashboard == null) {
            halDashboard = HalDashboard.getInstance(telemetry);
        }
        halDashboard.resetTelemetryForHalDashboard();
        return halDashboard;
    }

    private enum Alliance {
        RED_ALLIANCE,
        BLUE_ALLIANCE
    }   //enum Alliance
}
