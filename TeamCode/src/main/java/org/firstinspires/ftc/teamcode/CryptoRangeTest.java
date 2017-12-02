package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.LinkedList;
import java.util.List;

/**
 * Created by vvestin on 11/22/17.
 */

@TeleOp (name = "!!!!", group = "autonomous")
public class CryptoRangeTest extends OpMode implements GameConstants {
    private DcMotor motor1, motor2, motor3, motor4;
    private ModernRoboticsI2cRangeSensor rangeSensor;
    private boolean getData = true;
    private int ColumnPassed = 0;
    double distance;
    double prevdistance;
    List<Integer> readings;
    public void init() {

        readings = new LinkedList<Integer>();

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
        if (gamepad1.b) {
            getData = false;
        }
        if (getData) {
            double distance1 = rangeSensor.getDistance(DistanceUnit.CM);
            double distance2 = rangeSensor.getDistance(DistanceUnit.CM);
            double distance3 = rangeSensor.getDistance(DistanceUnit.CM);

            if (distance1 <= distance2 && distance1 <= distance3) {
                distance = distance1;
            }
            else if (distance2 <= distance1 && distance2 <= distance3) {
                distance = distance2;
            }
            else if (distance3 <= distance2 && distance3 <= distance1) {
                distance = distance3;
            }
            if (distance > 255) {
                return;
            }
            readings.add((int) distance);
            telemetry.addData("distance", rangeSensor.getDistance(DistanceUnit.CM));
            if (distance >= prevdistance + 5) {
                ColumnPassed += 1;
            }
            prevdistance = distance;
            telemetry.addData("Distance1", distance1);
            telemetry.addData("Distance2", distance2);
            telemetry.addData("Distance3", distance3);
            telemetry.addData("Columns Passed:", ColumnPassed);
        }
        else {
            telemetry.addData("Columns Passed:", ColumnPassed);
            telemetry.addData("Readings:", readings);
        }
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
