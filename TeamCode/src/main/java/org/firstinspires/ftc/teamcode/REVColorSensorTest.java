package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;

/**
 * Created by vvestin on 1/25/18.
 */
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "REVColorSensor")
public class REVColorSensorTest extends OpMode {
    private ColorSensor colorSensor;

    public void init() {
        colorSensor = hardwareMap.colorSensor.get("jewelColor");

    }

    public void loop() {
        telemetry.addData("red", colorSensor.red());
        telemetry.addData("green", colorSensor.green());
        telemetry.addData("blue", colorSensor.blue());
    }
}
