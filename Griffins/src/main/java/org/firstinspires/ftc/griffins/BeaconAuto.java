package org.firstinspires.ftc.griffins;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

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
            color = RobotHardware.BeaconState.BLUE;
            notColor = RobotHardware.BeaconState.RED;
        } else if (alliance == Alliance.RED_ALLIANCE) {
            toWall = AutoFunctions.TurnDirection.LEFT;
            awayFromWall = AutoFunctions.TurnDirection.RIGHT;
            color = RobotHardware.BeaconState.RED;
            notColor = RobotHardware.BeaconState.BLUE;
        }

        autoFunctions.setAlliance(color);
        autoFunctions.setAlliance(null);
        telemetry.log().add("versioning 10");

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
        autoFunctions.driveStraightPID(51, AutoFunctions.DriveStraightDirection.FORWARD, 3, true);
        telemetry.log().add("Arrived at beacon wall");
        telemetry.update();

        autoFunctions.setAlliance(color);

        autoFunctions.driveStraightPID(2, AutoFunctions.DriveStraightDirection.BACKWARD, 1);

        angle -= autoFunctions.getZAngle();

        //"parallel parking"
        int gyroHeading = hardware.getTurretGyro().getIntegratedZValue();
        autoFunctions.twoWheelTurnPID(22 + (alliance == Alliance.RED_ALLIANCE ? -1 : 1) * angle, awayFromWall, 2);

        hardware.registerBeaconColorSensors();
        hardware.registerLoaderColorSensor();

        autoFunctions.driveStraightPID(22, AutoFunctions.DriveStraightDirection.FORWARD, 1.5, true);
        autoFunctions.wallPIDDrive(22, AutoFunctions.DriveStraightDirection.FORWARD, toWall, 2);

        //autoFunctions.twoWheelTurnPID(20, AutoFunctions.TurnDirection.RIGHT, .3, true); //timer out

        hardware.stopDrive();

        autoFunctions.scanForBeacon(AutoFunctions.DriveStraightDirection.FORWARD, toWall);

        hardware.setLoaderPower(0);

        setDrivePower(0.2, 0.1);

        autoFunctions.autoLoadingSleep(200);
        hardware.stopDrive();

        autoFunctions.pushBeacon(color);

        setDrivePower(-0.3, 0.3);

        autoFunctions.autoLoadingSleep(300);
        hardware.stopDrive();

        //autoFunctions.twoWheelTurnPID(45, AutoFunctions.TurnDirection.LEFT, 0.5, true); //timer out
        autoFunctions.wallPIDDrive(45, AutoFunctions.DriveStraightDirection.BACKWARD, toWall, 2);

        autoFunctions.scanForBeacon(AutoFunctions.DriveStraightDirection.BACKWARD, toWall);

        setDrivePower(-0.2, -0.1);

        autoFunctions.autoLoadingSleep(300);
        hardware.stopDrive();

        hardware.getIntake().setPower(0);
        hardware.getTurretRotation().setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hardware.getTurretRotation().setTargetPosition((int) (hardware.getTurretRotation().getCurrentPosition() +
                (toWall == AutoFunctions.TurnDirection.LEFT ? 1 : -1) * RobotHardware.ENCODER_COUNTS_PER_TURRET_DEGREE * 38));
        hardware.getTurretRotation().setPower(.5);

        autoFunctions.pushBeacon(color, true);

        setDrivePower(0.25, 0.4);
        sleep(900);
        setDrivePower(0, -0.6);
        sleep(800);
        setDrivePower(-0.5, -0.3);
        sleep(2000);
        hardware.stopDrive();

    }

    // this methods mirrors driving commands, to use, enter powers for the blue auto,
    // and the method will mirror the driving power when the alliance is red.
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
