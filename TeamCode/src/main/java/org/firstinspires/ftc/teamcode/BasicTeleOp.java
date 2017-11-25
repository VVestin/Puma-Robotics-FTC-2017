package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * Created by vvestin on 9/23/17.
 */
@TeleOp(name="BasicTeleOp",group="D")
public class BasicTeleOp extends OpMode {

    private DcMotor motor1;
    private DcMotor motor2;
    private DcMotor motor3;
    private DcMotor motor4;

    private DcMotor lifter;
    private Servo s1;
    private Servo s2;

    private OrientationSensor orientationSensor;

    private double pos = 1;

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

        lifter = hardwareMap.dcMotor.get("lift");

        s1 = hardwareMap.servo.get("s1");
        s2 = hardwareMap.servo.get("s2");

        orientationSensor = new OrientationSensor(hardwareMap);
    }

    @Override
    public void loop() {
        double heading = orientationSensor.getOrientation();
        if (gamepad1.a) {
            pos -= .03;
        }
        if(gamepad1.b) {
            pos += .03;
        }
        s1.setPosition(pos);
        s2.setPosition(1 - pos);

        lifter.setPower(0);
        if(gamepad1.dpad_up){
            lifter.setPower(.5);
        }
        if(gamepad1.dpad_down){
            lifter.setPower(-.5);
        }

        motor1.setPower(0);
        motor2.setPower(0);
        motor3.setPower(0);
        motor4.setPower(0);
        if (gamepad1.left_trigger > 0.05) {
            motor1.setPower(-Math.pow(gamepad1.left_trigger, 5));
            motor2.setPower(-Math.pow(gamepad1.left_trigger, 5));
            motor3.setPower(Math.pow(gamepad1.left_trigger, 5));
            motor4.setPower(Math.pow(gamepad1.left_trigger, 5));
            return;
        }
        if (gamepad1.right_trigger > 0.05) {
            motor1.setPower(Math.pow(gamepad1.right_trigger, 5));
            motor2.setPower(Math.pow(gamepad1.right_trigger, 5));
            motor3.setPower(-Math.pow(gamepad1.right_trigger, 5));
            motor4.setPower(-Math.pow(gamepad1.right_trigger, 5));
            return;
        }
       // Equations:
       // x'=xcos0+ysin0
       // y'=-xsin0+ycos0
        double rawx = gamepad1.right_stick_x;
        double rawy = -gamepad1.right_stick_y;

        double x = Math.pow(rawx, 7);// rawx * Math.cos(Math.toRadians(-heading)) - rawy * Math.sin(Math.toRadians(-heading));
        double y = Math.pow(rawy, 7);// rawx * Math.sin(Math.toRadians(-heading)) + rawy * Math.cos(Math.toRadians(-heading));

        if (x != 0 || y != 0) {
            double n = ((x + y) / 2.0); // n is the power of the motors in the +x +y direction
            double m = ((x - y) / 2.0); // m is the power of the motors in the +x -y direction
            motor1.setPower(m);
            motor2.setPower(n);
            motor3.setPower(m);
            motor4.setPower(n);
            telemetry.addData("m", m);
            telemetry.addData("n", n);
        }

        lifter.setPower(0);
        if (gamepad1.b && pos > .2) {
            pos -= .05;
        }
        if(gamepad1.a && pos < 1) {
            pos += .05;
        }

        s1.setPosition(pos);
        s2.setPosition(1 - pos);
        if(gamepad1.dpad_up){
            lifter.setPower(.5);
        }
        if(gamepad1.dpad_down){
            lifter.setPower(-.5);
        }

        telemetry.addData("heading is: ", heading);
        telemetry.addData("position is: ", pos);
    }
}