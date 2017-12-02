package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by vvestin on 12/2/17.
 */
@TeleOp(name="Relic Grabber Test",group="test")
public class RelicGrabberTest extends OpMode {

    private double clawPos = .5;
    private double pivotPos = .5;

    private Servo relicClaw;
    private Servo relicPivot;
    private DcMotor extender;

    @Override
    public void init() {
        relicClaw = hardwareMap.servo.get("relicClaw");
        relicPivot = hardwareMap.servo.get("relicPivot");

        extender = hardwareMap.dcMotor.get("extender");
    }

    @Override
    public void loop() {
        relicClaw.setPosition(clawPos);
        relicPivot.setPosition(pivotPos);

        if (gamepad1.y && clawPos < 1) {
            clawPos += 0.01;
        }
        if (gamepad1.x && clawPos > 0) {
            clawPos -= 0.01;
        }

        if (gamepad1.left_bumper && pivotPos < 1) {
            pivotPos += 0.01;
        }
        if (gamepad1.right_bumper && pivotPos > 0) {
            pivotPos -= 0.01;
        }

        if (gamepad1.dpad_right) {
            extender.setPower(0.2);
        }
        else {
            extender.setPower(0);
        }
    }
}
