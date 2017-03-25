package org.firstinspires.ftc.griffins;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.griffins.Testing.WallApproachTest;

/**
 * Created by David on 3/25/2017.
 */

public abstract class BeaconAuto extends LinearOpMode {
    protected Alliance alliance;
    private RobotHardware hardware;
    private AutoFunctions autoFunctions;

    private AutoFunctions.TurnDirection toWall;
    private AutoFunctions.TurnDirection awayFromWall;

    private RobotHardware.BeaconState color;
    private RobotHardware.BeaconState notColor;

    @Override
    public void runOpMode() throws InterruptedException {
        hardware = new RobotHardware();
        hardware.initialize(hardwareMap);
        autoFunctions = new AutoFunctions(hardware, this);

        telemetry.log().add("Alliance is " + alliance);
        if (alliance == Alliance.BLUE_ALLIANCE) {
            toWall = AutoFunctions.TurnDirection.RIGHT;
            awayFromWall = AutoFunctions.TurnDirection.LEFT;
        } else if (alliance == Alliance.RED_ALLIANCE) {
            toWall = AutoFunctions.TurnDirection.LEFT;
            awayFromWall = AutoFunctions.TurnDirection.RIGHT;
        }

        waitForStart();

        while (opModeIsActive() && hardware.getTurretGyro().isCalibrating()) ;

        //shoot two particles
        autoFunctions.shoot();
        telemetry.log().add("Finished Shooting");
        telemetry.update();

        double angle = autoFunctions.getZAngle();

        hardware.getIntake().setPower(1);
        sleep(500);
        hardware.getIntake().setPower(0);

        //drive toward beacon wall
        autoFunctions.driveStraightPID(64, AutoFunctions.DriveStraightDirection.FORWARD, 3, true);
        telemetry.log().add("Arrived at beacon wall");
        telemetry.update();

        //autoFunctions.driveStraightPID(1, AutoFunctions.DriveStraightDirection.BACKWARD, 1);

        angle -= autoFunctions.getZAngle();

        //"parallel parking"
        if (alliance == Alliance.BLUE_ALLIANCE) {
            WallApproachTest.blueWallApproach(hardware, autoFunctions, this, (int) angle);
            color = RobotHardware.BeaconState.BLUE;
            notColor = RobotHardware.BeaconState.RED;
        }
        if (alliance == Alliance.RED_ALLIANCE) {
            WallApproachTest.redWallApproach(hardware, autoFunctions, this, (int) angle);
            color = RobotHardware.BeaconState.RED;
            notColor = RobotHardware.BeaconState.BLUE;
        }

        if (hardware.findParticleColor() == notColor) {
            hardware.setLoaderPower(-1);
        }

        autoFunctions.scanForBeacon(AutoFunctions.DriveStraightDirection.FORWARD);

        hardware.setLoaderPower(0);

        setDrivePower(-0.2, -0.1);

        sleep(200);

        setDrivePower(-0.3, 0.3);

        sleep(200);
        hardware.stopDrive();

        autoFunctions.pushBeacon(color);

        //autoFunctions.twoWheelTurnPID(45, AutoFunctions.TurnDirection.LEFT, 0.5, true); //timer out
        autoFunctions.wallPIDDrive(40, AutoFunctions.DriveStraightDirection.BACKWARD, toWall, 2);

        autoFunctions.scanForBeacon(AutoFunctions.DriveStraightDirection.BACKWARD);

        setDrivePower(-0.2, -0.1);

        sleep(200);
        hardware.stopDrive();

        autoFunctions.pushBeacon(color);

        setDrivePower(0.2, 0.3);
        sleep(800);
        setDrivePower(0, -0.6);
        sleep(800);
        setDrivePower(-0.5, -0.3);
        sleep(2000);
        hardware.stopDrive();

    }

    public void setDrivePower(double power1, double power2) {
        if (alliance == Alliance.BLUE_ALLIANCE)
            hardware.setDrivePower(power1, power2);
        if (alliance == Alliance.RED_ALLIANCE)
            hardware.setDrivePower(power2, power1);
    }

    protected enum Alliance {
        RED_ALLIANCE,
        BLUE_ALLIANCE
    }   //enum Alliance
}
