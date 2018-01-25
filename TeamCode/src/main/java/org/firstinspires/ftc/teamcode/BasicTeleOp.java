package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

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
    private ColorSensor jewelSensor;

    private double pos = 1;
    private double pivotPos = .38;
    private double clawPos = .5;
    private double jewelBarPos = 0.24;
    private boolean slow;

    private Servo USpivot;
    private Servo relicClaw;
    private Servo relicPivot;
    private Servo jewelBar;
    private DcMotor extender;

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

        relicClaw = hardwareMap.servo.get("relicClaw");
        relicPivot = hardwareMap.servo.get("relicPivot");

        extender = hardwareMap.dcMotor.get("extender");
        extender.setDirection(DcMotorSimple.Direction.REVERSE);

        USpivot= hardwareMap.servo.get("usp");

        jewelBar = hardwareMap.servo.get("jewel");
        jewelSensor = hardwareMap.colorSensor.get("jewelsensor");

        slow = false;
    }

    public void start() {
        USpivot.setPosition(1);
    }

    @Override
    public void loop() {
        double heading = orientationSensor.getOrientation();

        motor1.setPower(0);
        motor2.setPower(0);
        motor3.setPower(0);
        motor4.setPower(0);
        if (gamepad1.left_trigger > 0.05) {
            double trigger = gamepad1.left_trigger;
            if (slow) trigger /= 1.2;
            motor1.setPower(-Math.pow(trigger, 5));
            motor2.setPower(-Math.pow(trigger, 5));
            motor3.setPower(Math.pow(trigger, 5));
            motor4.setPower(Math.pow(trigger, 5));
            return;
        }
        if (gamepad1.right_trigger > 0.05) {
            double trigger = gamepad1.right_trigger;
            if (slow) trigger /= 1.2;
            motor1.setPower(Math.pow(trigger, 5));
            motor2.setPower(Math.pow(trigger, 5));
            motor3.setPower(-Math.pow(trigger, 5));
            motor4.setPower(-Math.pow(trigger, 5));
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
            double n = ((x + y) / Math.sqrt(2.0)); // n is the power of the motors in the +x +y direction
            double m = ((x - y) / Math.sqrt(2.0)); // m is the power of the motors in the +x -y direction
            if (slow) {
                m /= 2;
                n /= 2;
            }
            motor1.setPower(m);
            motor2.setPower(n);
            motor3.setPower(m);
            motor4.setPower(n);
        }

        lifter.setPower(0);

        if (((gamepad1.b && pos > .2) || (gamepad2.b && pos>0.2)) && pos > .2) {
            pos -= .1;
        }
        if (((gamepad1.a && pos < 1) || (gamepad2.a && pos<1)) && pos < .7) {
            pos += .1;
        }

        s1.setPosition(pos);
        s2.setPosition(1 - pos);
        lifter.setPower(0);
        if(gamepad1.dpad_up || gamepad2.dpad_up){
            lifter.setPower(.5);
        }
        if(gamepad1.dpad_down || gamepad2.dpad_down){
            lifter.setPower(-.5);
        }

        telemetry.addData("heading is: ", heading);
        telemetry.addData("position is: ", pos);
        relicClaw.setPosition(pivotPos);
        relicPivot.setPosition(clawPos);


        if (gamepad2.right_bumper && pivotPos < .5) {
            pivotPos += 0.016; // pivot down
        }
        if (gamepad2.left_bumper && pivotPos > 0) {
            pivotPos -= 0.016; // pivot up
        }

        if (gamepad2.y && clawPos < 0.5) {
            clawPos += 0.08; // close claw
        }
        if (gamepad2.x && clawPos > 0) {
            clawPos -= 0.08; // open claw
        }

        if (gamepad2.dpad_right) {
            extender.setPower(0.6);
        }
        else {
            extender.setPower(0);
        }
        if (gamepad1.dpad_left) {
            jewelBarPos = .24;
        } else if (gamepad1.dpad_right) {
            jewelBarPos = .75;
        }

        telemetry.addData("red", jewelSensor.red());
        telemetry.addData("jewelBar", jewelBarPos);
        jewelBar.setPosition(jewelBarPos);

        telemetry.addData("pivotPos is: ", pivotPos);
        telemetry.addData("clawPos is: ", clawPos);

        if(gamepad1.x) {
            slow = true;
        }
        if(gamepad1.y) {
            slow = false;
        }
    }
}