package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by vvestin on 9/30/17.
 */
@TeleOp(name = "grabber", group = "MRI")
public class GrabberTest extends OpMode {
    private Servo s1;
    private Servo s2;
    @Override
    public void init() {
        s1 = hardwareMap.servo.get("s1");
        s2 = hardwareMap.servo.get("s2");
    }
    @Override
    public void loop() {
        double pos = 0.5;

        if(gamepad1.a == true) {
            s1.setPosition(pos + 0.05);
            s2.setPosition(pos - 0.05);
        }
        if(gamepad1.b == true){
            s1.setPosition(pos - 0.05);
            s2.setPosition(pos + 0.05);
        }
    }
}
