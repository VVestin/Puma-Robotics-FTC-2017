package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by vvestin on 1/26/18.
 */

@TeleOp(name = "jewelKnockerTest")
public class JewelKnockerTest extends OpMode {

    private Servo jewelKnocker;
    private double pos = .5;

    public void init() {
        jewelKnocker = hardwareMap.servo.get("jewel");
    }

    public void loop() {
        jewelKnocker.setPosition(pos);
        telemetry.addData("pos", pos);

        if (gamepad1.dpad_right) {
            pos += .001;
        }
        if (gamepad1.dpad_left) {
            pos -= .001;
        }
    }
}
