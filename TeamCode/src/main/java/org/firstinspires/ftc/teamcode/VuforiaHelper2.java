package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class VuforiaHelper2 {
    private static int KEY_COLUMN;

    public VuforiaHelper2(HardwareMap hardwareMap) { }

    public void start() {
        KEY_COLUMN = (int) (3 * Math.random() + 1);
    }

    public void loop() { }

    public int getKeyColumn() {
        return KEY_COLUMN;
    }
}