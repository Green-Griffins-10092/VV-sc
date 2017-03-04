package org.firstinspires.ftc.griffins.Testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.griffins.AutoFunctions;
import org.firstinspires.ftc.griffins.AutoFunctions.TurnDirection;
import org.firstinspires.ftc.griffins.Navigation.PIDRate;
import org.firstinspires.ftc.griffins.RobotHardware;

/**
 * Created by David on 12/20/2016.
 */
@Autonomous(group = "test")
@Disabled
public class PIDShootingTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        RobotHardware robot = new RobotHardware();
        PIDRate rate = new PIDRate(robot);
        robot.initialize(hardwareMap);
        AutoFunctions autoFunctions = new AutoFunctions(robot, this);

        waitForStart();

        String string = autoFunctions.shootPID(.77);
        log("finished shot 1");
        FileOutput.outputFile("shot1.csv", string);
        sleep(2000);
        string = autoFunctions.shootPID(.77);
        FileOutput.outputFile("shot2.csv", string);
        log("finished shot 2");
        sleep(2000);
        string = autoFunctions.shootPID(.77);
        FileOutput.outputFile("shot3.csv", string);
        log("finished shot 3");
    }

    public void log(String message) {
        telemetry.log().add(message);
        telemetry.update();
    }
}
