package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * Created by vvestin on 9/23/17.
 */
@TeleOp(name="BasicTeleOp",group="Motor")
public class BasicTeleOp extends OpMode {

    private DcMotor motor1;
    private DcMotor motor2;
    private DcMotor motor3;
    private DcMotor motor4;

    BNO055IMU imu;
    Orientation angles;
    Acceleration gravity;

    @Override
    public void init() {
        motor1 = hardwareMap.dcMotor.get("w1");
        motor2 = hardwareMap.dcMotor.get("w2");
        motor3 = hardwareMap.dcMotor.get("w3");
        motor4 = hardwareMap.dcMotor.get("w4");

        motor2.setDirection(DcMotorSimple.Direction.REVERSE);
        motor3.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop() {
        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        gravity  = imu.getGravity();

        telemetry.addData("Heading",angles.firstAngle);

        motor1.setPower(0);
        motor2.setPower(0);
        motor3.setPower(0);
        motor4.setPower(0);
        if (gamepad1.left_trigger > 0.05) {
            motor1.setPower(gamepad1.left_trigger);
            motor2.setPower(-1*gamepad1.left_trigger);
            motor3.setPower(-1*gamepad1.left_trigger);
            motor4.setPower(gamepad1.left_trigger);
            return;
        }
        if (gamepad1.right_trigger > 0.05) {
            motor1.setPower(-1*gamepad1.right_trigger);
            motor2.setPower(gamepad1.right_trigger);
            motor3.setPower(gamepad1.right_trigger);
            motor4.setPower(-1*gamepad1.right_trigger);
            return;
        }

        double x = gamepad1.right_stick_x;
        double y = gamepad1.right_stick_y;
        double newx = x*Math.cos(angle);
        double m;
        double n;
        if (x != 0 || y != 0) {
            n = ((x + y)/2.0)*angles.firstAngle;
            m = -((x - y)/2.0)*angles.firstAngle;
            motor1.setPower(m);
            motor2.setPower(n);
            motor3.setPower(m);
            motor4.setPower(n);
        }
    }
}