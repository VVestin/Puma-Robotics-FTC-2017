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

    private Servo s1; // TODO give these more meaningful identifiers
    private Servo s2;
    private CRServo cr1;
    private CRServo cr2;

    private OrientationSensor orientationSensor;

    private double pos = 0.5;

    @Override
    public void init() {
        motor1 = hardwareMap.dcMotor.get("w1");
        motor2 = hardwareMap.dcMotor.get("w2");
        motor3 = hardwareMap.dcMotor.get("w3");
        motor4 = hardwareMap.dcMotor.get("w4");

        motor1.setDirection(DcMotorSimple.Direction.REVERSE);
        motor2.setDirection(DcMotorSimple.Direction.REVERSE);

        s1 = hardwareMap.servo.get("s1");
        s2 = hardwareMap.servo.get("s2");

        cr1=hardwareMap.crservo.get("cr1");
        cr2=hardwareMap.crservo.get("cr2");

        orientationSensor = new OrientationSensor(hardwareMap);

    }

    @Override
    public void loop() {
        double heading = orientationSensor.getOrientation();

        motor1.setPower(0);
        motor2.setPower(0);
        motor3.setPower(0);
        motor4.setPower(0);
        if (gamepad1.left_trigger > 0.05) {
            motor1.setPower(-gamepad1.left_trigger);
            motor2.setPower(-gamepad1.left_trigger);
            motor3.setPower(gamepad1.left_trigger);
            motor4.setPower(gamepad1.left_trigger);
            return;
        }
        if (gamepad1.right_trigger > 0.05) {
            motor1.setPower(gamepad1.right_trigger);
            motor2.setPower(gamepad1.right_trigger);
            motor3.setPower(-gamepad1.right_trigger);
            motor4.setPower(-gamepad1.right_trigger);
            return;
        }
       // Equations:
       // x'=xcos0+ysin0
       // y'=-xsin0+ycos0
        double rawx = gamepad1.right_stick_x;
        double rawy = gamepad1.right_stick_y;
        double x = rawx * Math.cos(Math.toRadians(-heading)) - rawy * Math.sin(Math.toRadians(-heading));
        double y = rawx * Math.sin(Math.toRadians(-heading)) + rawy * Math.cos(Math.toRadians(-heading));
        if (x != 0 || y != 0) {
            double n = ((x + y) / 2.0); // n is the power of the motors in the +x +y direction
            double m = -((x - y) / 2.0); // m is the power of the motors in the +x -y direction
            motor1.setPower(m);
            motor2.setPower(n);
            motor3.setPower(m);
            motor4.setPower(n);
        }
        if (gamepad1.a) {
            pos -= .01;
        }
        if(gamepad1.b) {
            pos += .01;
        }

        s1.setPosition(pos);
        s2.setPosition(1 - pos);

        cr1.setPower(0);
        cr2.setPower(0);

        if(gamepad1.dpad_up){
            cr1.setPower(-1);
            cr2.setPower(1);
        }
        if(gamepad1.dpad_down) {
            cr2.setPower(-1);
            cr1.setPower(1);
        }
        if(gamepad1.x){
            cr1.setPower(0);
            cr2.setPower(0);
        }
        telemetry.addData("heading is: ", heading);
        telemetry.addData("position is: ", pos);
    }
}