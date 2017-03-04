package org.firstinspires.ftc.griffins;

/**
 * Created by David on 2/12/2017.
 */
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Blue Teleop", group = "Competition")
public class BlueTeleOp extends TeleOp {
    @Override
    public void init() {
        super.init();
        alliance = RobotHardware.BeaconState.BLUE;
        telemetry.log().add("Alliance is " + alliance);
    }
}
