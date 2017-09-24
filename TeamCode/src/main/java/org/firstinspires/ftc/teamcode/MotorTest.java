package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by vvestin on 9/23/17.
 */
@TeleOp(name = "Servo", group = "MRI")
public class MotorTest extends OpMode{

    private CRServo motor;

    @Override
    public void init() {
        motor = hardwareMap.crservo.get("m1");
    }

    @Override
    public void loop() {
        motor.setPower(1);
    }
}
