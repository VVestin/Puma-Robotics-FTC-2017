package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by vvestin on 12/23/17.
 */

@TeleOp(name="USPivotTest")
public class PivotServoTest extends OpMode {
    private Servo pivot;
    private double pos;

    public void init() {
        pivot = hardwareMap.servo.get("usp");
        pos = .5;

    }

    public void loop() {
        if (gamepad1.a) {
            pos += .01;
        }
        if (gamepad1.b) {
            pos -= .01;
        }
        pivot.setPosition(pos);
        telemetry.addData("position", pos);
    }

}
