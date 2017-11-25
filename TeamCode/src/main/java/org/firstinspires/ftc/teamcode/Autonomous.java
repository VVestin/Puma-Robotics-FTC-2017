package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Stack;
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Autonomous" , group = "AAAAAARP")
public class Autonomous extends OpMode implements GameConstants {

    public static final String TAG = "Vuforia VuMark Sample";
    public static final String VUFORIA_KEY = "AdrNx7L/////AAAAGWscSRJwJ0For7OwbugY3F"
            + "d0I3f+mAuH+BAHpz7UBuNJnU+QudRFM8gzxBh+mZcuiwi2TStZTxHuDQvVJHER5zuUmh7"
            + "X6dr/7uEnPy+OBd72HjBc2gM+w7DNmcBhY8SmEgLRlzhI4dRCAmjADeVQd9c/vTTyqWSY"
            + "dy7F2fE2eQbSoXKyKN1uFV6P6lN3NlHSazLOaniTLpAQQlbOwb9S2KxXy7PQK1ZBAmWMd"
            + "Hb5jwAXaqz+HXMPBez6/7behYzk1eu4a/0hFZ6jWo9Khoc9MRrhmCac0SCzmNRjfD8Y9Q"
            + "61EtWvmo+WlbyzFJUNsZbND80BXAKaOWXvCAsdCo58qGtmVr36Bau5iljOe5HBbvov";

    // Actuators
    private DcMotor motor1;
    private DcMotor motor2;
    private DcMotor motor3;
    private DcMotor motor4;

    // Sensors
    private ModernRoboticsI2cRangeSensor rangeSensor;
    private OrientationSensor orientationSensor;
    private VuforiaHelper2 vuforia;

    // State machine
    private State state;
    private Stack<State> nextStates;

    // State machine variables
    private double delay;
    private double delayStart;

    // Autonomous variables
    private int objectCount;
    private boolean seeColumn;
    private int keyColumn;
    private double walldistance;
    private int numReadings;

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

        orientationSensor = new OrientationSensor(hardwareMap);

        state = State.START;
        nextStates = new Stack<State>();

        vuforia = new VuforiaHelper2(hardwareMap);
    }

    public void start() {
        vuforia.start();
    }

    public void loop() {
        vuforia.loop();
        if (vuforia.getKeyColumn() != 0) {
            keyColumn = vuforia.getKeyColumn();
        }

        double heading = orientationSensor.getOrientation();
        double distance = rangeSensor.getDistance(DistanceUnit.CM);


        if (distance > 200 || distance == 0) {
            return;
        }

        telemetry.addData("range", distance);

        switch (state) {
            case START:
                if (TOP_FIELD) {
                    state = State.TOP_FIELD;
                } else {
                    state = State.BOTTOM_FIELD;
                }
                break;
            case TOP_FIELD:
                state = State.COLUMN_COUNTING;
                break;
            case BOTTOM_FIELD:
                move(0, 0.4, 0);
                if (distance < 38) {
                    move(0, 0, 0);
                    state = State.START_COLUMN_COUNTING;
                }
                break;
            case START_COLUMN_COUNTING:
                walldistance += distance;
                numReadings++;
                if (numReadings == 5) {
                    walldistance /= numReadings;
                    telemetry.addData("Wall Distance", walldistance);
                    state = State.COLUMN_COUNTING;
                }
                break;
            case COLUMN_COUNTING:
                // Oriented with sensor facing Cryptobox
                if (RED_TEAM) {
                    move(-.4, 0, 0);
                } else {
                    move(.4, 0, 0);
                }

                if (distance < walldistance - 2 && !seeColumn) {
                    seeColumn = true;
                }
                if (distance > walldistance - 1 && seeColumn) {
                    seeColumn = false;
                    objectCount++;
                }

                telemetry.addData("Current distance is ", distance);
                telemetry.addData("Objects passed: ", objectCount);
                if (keyColumn == objectCount) {
                    if (RED_TEAM) {
                        move(.4, 0, 0);
                    } else {
                        move(-.4, 0, 0);
                    }
                    delay = 0.7;
                    nextStates.push(State.PUSH_GLYPH_IN);
                    state = State.DELAY;
                }
                break;
            case PUSH_GLYPH_IN:
                move(0, 0.4, 0);
                delay = 0.8;
                nextStates.push(State.STOP);
                state = State.DELAY;
                break;
            case DELAY:
                delayStart = time;
                state = State.DELAY_LOOP;
                break;
            case DELAY_LOOP:
                if (time > delayStart + delay)
                    state = nextStates.pop();
                break;
            case STOP:
                move(0, 0, 0);
                break;
        }
        telemetry.addData("State", state);
    }

    // double x = gamepad1.right_stick_x;
    // double y = gamepad1.right_stick_y;
    // double dir = gamepad1.right_trigger or gamepad1.left_trigger;
    public void move(double x, double y, double dir) {
        double n = ((x + y) / 2.0); // n is the power of the motors in the +x +y direction
        double m = ((x - y) / 2.0); // m is the power of the motors in the +x -y direction
        motor1.setPower(m);
        motor2.setPower(n);
        motor3.setPower(m);
        motor4.setPower(n);
    }
}