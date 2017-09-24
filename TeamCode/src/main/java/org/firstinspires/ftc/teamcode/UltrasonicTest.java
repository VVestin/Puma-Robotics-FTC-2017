package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;

/**
 * Created by vvestin on 9/23/17.
 */

public class UltrasonicTest extends OpMode{

    UltrasonicSensor sonic;

    @Override
    public void init() {
        sonic = hardwareMap.ultrasonicSensor.get("sonic");
    }

    @Override
    public void loop() {
        sonic.getUltrasonicLevel();
        telemetry.addData("What is the value or the sensor?", sonic);
        sonic.status();
    }
}
