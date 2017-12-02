package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Created by vvestin on 10/14/17.
 */
@TeleOp(name="IndivMotorTest",group="D")
public class IndivMotorTest extends OpMode {
    private DcMotor motor1;
    private DcMotor motor2;
    private DcMotor motor3;
    private DcMotor motor4;

    @Override
    public void init() {
        motor1 = hardwareMap.dcMotor.get("w1");
        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor1.setDirection(DcMotorSimple.Direction.REVERSE);

        motor2 = hardwareMap.dcMotor.get("w2");
        motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor2.setDirection(DcMotorSimple.Direction.REVERSE);

        motor3 = hardwareMap.dcMotor.get("w3");
        motor3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motor4 = hardwareMap.dcMotor.get("w4");
        motor4.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motor1.setDirection(DcMotorSimple.Direction.REVERSE);
        motor2.setDirection(DcMotorSimple.Direction.REVERSE);

    }
    @Override
    public void loop() {
        motor1.setPower(0);
        motor2.setPower(0);
        motor3.setPower(0);
        motor4.setPower(0);
        if(gamepad1.a){
            motor1.setPower(.5);
        }
        if(gamepad1.b){
            motor2.setPower(.5);
        }
        if(gamepad1.x){
            motor3.setPower(.5);
        }
        if(gamepad1.y){
            motor4.setPower(.5);
        }
        telemetry.addData("encoder 1: ", motor1.getCurrentPosition());
        telemetry.addData("encoder 2: ", motor2.getCurrentPosition());
        telemetry.addData("encoder 3: ", motor3.getCurrentPosition());
        telemetry.addData("encoder 4: ", motor4.getCurrentPosition());
    }
}
