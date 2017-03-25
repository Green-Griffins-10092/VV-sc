package org.firstinspires.ftc.griffins;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by David on 12/7/2016.
 */
@Autonomous(name = "Blue Beacon Auto", group = "Competition")
public class BlueAuto extends BeaconAuto {
    private RobotHardware hardware;
    private AutoFunctions autoFunctions;

    @Override
    public void runOpMode() throws InterruptedException {
        alliance = BeaconAuto.Alliance.BLUE_ALLIANCE;
        super.runOpMode();
    }
}