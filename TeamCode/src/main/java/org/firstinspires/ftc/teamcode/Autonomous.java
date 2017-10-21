package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Created by vvestin on 10/21/17.
 */
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Autonomous", group = "Auto")
public class Autonomous extends OpMode {
    private DcMotor motor1;
    private DcMotor motor2;
    private DcMotor motor3;
    private DcMotor motor4;
    private State state;
    private int keyColumn=1;
    ModernRoboticsI2cRangeSensor rangeSensor;
    int objectCount;
    boolean seeColumn = false;

    public void init() {
        motor1 = hardwareMap.dcMotor.get("w1");
        motor2 = hardwareMap.dcMotor.get("w2");
        motor3 = hardwareMap.dcMotor.get("w3");
        motor4 = hardwareMap.dcMotor.get("w4");

        motor1.setDirection(DcMotorSimple.Direction.REVERSE);
        motor2.setDirection(DcMotorSimple.Direction.REVERSE);
        rangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "sensor_range");

        state = State.COLUMN_COUNTING;

    }

    public void loop() {
        switch (state) {
            case COLUMN_COUNTING:
                motor1.setPower(-0.2);
                motor2.setPower(-0.2);
                motor3.setPower(-0.2);
                motor4.setPower(-0.2);
                double distance = rangeSensor.getDistance(DistanceUnit.CM);
                if (distance > 200) {
                    return;
                }
                if (distance < 10 && seeColumn == false) {
                    seeColumn = true;
                }
                if (distance > 12 && seeColumn == true) {
                    seeColumn = false;
                    objectCount++;
                }
                telemetry.addData("Current distance is ", distance);
                telemetry.addData("Objects passed: ", objectCount);
                if(keyColumn==objectCount){
                    state=State.STOP;
                }
                break;
            case STOP:
                motor1.setPower(0);
                motor2.setPower(0);
                motor3.setPower(0);
                motor4.setPower(0);
                break;
        }

    }
}
