package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Created by vvestin on 11/22/17.
 */

public class CryptoRangeTest extends OpMode implements GameConstants {
    private DcMotor motor1, motor2, motor3, motor4;
    private ModernRoboticsI2cRangeSensor rangeSensor;

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

        rangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "sensor_range");
    }

    public void loop() {
        move(RED_TEAM ? -.3 : .3, 0, 0);
        telemetry.addData("distance", rangeSensor.getDistance(DistanceUnit.CM));
    }

    public void move(double x, double y, double dir) {
        double n = ((x + y) / 2.0); // n is the power of the motors in the +x +y direction
        double m = ((x - y) / 2.0); // m is the power of the motors in the +x -y direction
        motor1.setPower(m);
        motor2.setPower(n);
        motor3.setPower(m);
        motor4.setPower(n);
    }
}
