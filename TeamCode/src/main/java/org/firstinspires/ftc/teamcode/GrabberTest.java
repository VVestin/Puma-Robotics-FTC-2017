package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by vvestin on 9/30/17.
 */
@TeleOp(name = "grabber", group = "MRI")
public class GrabberTest extends OpMode {
    private Servo s1;
    private Servo s2;
    private DcMotor lifter;
    private double pos=.5;
    @Override
    public void init() {
        s1 = hardwareMap.servo.get("s1");
        s2 = hardwareMap.servo.get("s2");
        lifter=hardwareMap.dcMotor.get("lift");
    }
    @Override
    public void loop() {
        lifter.setPower(0);
        if (gamepad1.a) {
            pos -= .01;
        }
        if(gamepad1.b) {
            pos += .01;
        }

        s1.setPosition(pos);
        s2.setPosition(1 - pos);
        if(gamepad1.dpad_up){
            lifter.setPower(1);
        }
        if(gamepad1.dpad_down){
            lifter.setPower(-1);
        }
    }
}
