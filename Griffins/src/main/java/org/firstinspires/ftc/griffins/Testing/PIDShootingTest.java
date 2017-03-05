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
public class PIDShootingTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        RobotHardware robot = new RobotHardware();
        robot.initialize(hardwareMap);
        AutoFunctions autoFunctions = new AutoFunctions(robot, this);

        waitForStart();

       String string = autoFunctions.shootPIDtoString(7);
        log("finished shot");
        FileOutput.outputFile("liveShootingData.csv", string);

    }

    public void log(String message) {
        telemetry.log().add(message);
        telemetry.update();
    }
}
