package org.firstinspires.ftc.griffins.Testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by David on 11/26/2016.
 */

@TeleOp(name = "Gamepad Data Display", group = "test")
@Disabled
public class GamePadTest extends OpMode {
    @Override
    public void init() {

    }

    @Override
    public void loop() {
        telemetry.addData("gamepad 1", gamepad1);
        telemetry.addData("gamepad 2", gamepad2);
    }
}
