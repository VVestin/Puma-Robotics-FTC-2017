package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Created by vvestin on 10/7/17.
 */
@Autonomous(name = "Object Count", group = "Sensor")

public class ObjectCount  extends OpMode {
    ModernRoboticsI2cRangeSensor rangeSensor;
    int objectCount;
    boolean seeColumn=false;
    @Override
    public void init() {
        rangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "sensor_range");

        }
        public void loop(){
            double distance = rangeSensor.getDistance(DistanceUnit.CM);
            if(distance>200){
                return;
            }
            if(distance<10 && seeColumn==false){
                seeColumn=true;
            }
            if(distance>12 && seeColumn==true){
                seeColumn=false;
                objectCount++;
            }
            telemetry.addData("Current distance is ", distance);
            telemetry.addData("Objects passed: ", objectCount);
}
    }