package org.firstinspires.ftc.griffins;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by David on 12/7/2016.
 */
@Autonomous(name = "Red Beacon Auto", group = "Competition")
public class RedAuto extends BeaconAuto {

    @Override
    public void runOpMode() throws InterruptedException {
        alliance = Alliance.RED_ALLIANCE;
        super.runOpMode();
    }
}