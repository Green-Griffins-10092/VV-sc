package org.firstinspires.ftc.griffins;

/**
 * Created by David on 2/12/2017.
 */
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Red Teleop", group = "Competition")
public class RedTeleOp extends TeleOp {
    @Override
    public void init() {
        super.init();
        alliance = RobotHardware.BeaconState.RED;
        telemetry.log().add("Alliance is " + alliance);
    }
}
