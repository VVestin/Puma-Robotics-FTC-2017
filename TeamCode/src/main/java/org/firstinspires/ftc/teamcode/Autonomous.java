package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.LinkedList;
import java.util.List;
import java.util.Queue;
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
    private DcMotor lifter;
    private DcMotor extender;

    //Servo
    private Servo USpivot;
    private Servo s1, s2;

    // Sensors
    private ModernRoboticsI2cRangeSensor rangeSensor;
    private OrientationSensor orientationSensor;
    private VuforiaHelper vuforia;

    // State machine
    private State state;
    private Stack<State> nextStates;

    // State machine variables
    private double delay;
    private double delayStart;

    // Autonomous variables
    private Stack<Integer> readings;
    private static final int FILTER_BUFFER_LENGTH = 3;
    private Queue<Integer> distanceFilter;

    private int objectCount;
    private int keyColumn;
    private double distance = 255;

    private double prevdistance;
    private double distBeforeIncrease;
    private boolean distHasDecreased = false;


    private int readingDistNum;
    private double lastTime;
    private Stack<Integer> times = new Stack<Integer>();


    public static final double SPEED = 0.18;

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

        extender = hardwareMap.dcMotor.get("extender");
        extender.setDirection(DcMotorSimple.Direction.REVERSE);

        s1 = hardwareMap.servo.get("s1");
        s2 = hardwareMap.servo.get("s2");
        lifter=hardwareMap.dcMotor.get("lift");

        rangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range");

        orientationSensor = new OrientationSensor(hardwareMap);

        state = State.START;
        nextStates = new Stack<State>();

        vuforia = new VuforiaHelper(hardwareMap);

        USpivot = hardwareMap.servo.get("usp");
        USpivot.setPosition(1);

        readings = new Stack<Integer>();
        distanceFilter = new LinkedList<Integer>();
    }

    public void start() {
        vuforia.start();

        USpivot.setPosition(0.53);
    }

    public void loop() {
        if (keyColumn == 0) {
            vuforia.loop();
            keyColumn = vuforia.getKeyColumn();
            if (keyColumn == 0)
                return;
        }

        double heading = orientationSensor.getOrientation();
        telemetry.addData("heading", heading);

        switch (state) {
            case START:
                s1.setPosition(0.25);
                s2.setPosition(0.75);

                extender.setPower(0.3);

                delay = 1;
                state=State.DELAY;

                if (TOP_FIELD) {
                    nextStates.push(State.TOP_FIELD);
                } else {
                    nextStates.push(State.BOTTOM_FIELD);
                    motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    motor3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    motor4.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                }
                break;
            case TOP_FIELD:
                extender.setPower(0);
                state = State.START_COLUMN_COUNTING;
                break;
            case BOTTOM_FIELD:
                extender.setPower(0);
                move(0, SPEED + 0.1, 0);
                lifter.setPower(1);
                delay = 1.8;
                state = State.DELAY;
                nextStates.push(State.BOTTOM_FIELD_DIST);
                break;
            case BOTTOM_FIELD_DIST:
                motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motor3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motor4.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                move(0, SPEED + .1, 0);
                lifter.setPower(0);
                if (distance < 38) {
                    move(0, 0, 0);

                    delay = 1;
                    state = State.DELAY;
                    nextStates.push(State.BOTTOM_FIELD_DIST_ADJUST);
                    nextStates.push(State.GET_DISTANCE);
                }
                else {
                    state = State.GET_DISTANCE;
                    nextStates.push(State.BOTTOM_FIELD_DIST);
                }
                break;
            case BOTTOM_FIELD_DIST_ADJUST:
                if (distance < 33) {
                    move (0, -SPEED, 0);
                    state = State.GET_DISTANCE;
                    nextStates.push(State.BOTTOM_FIELD_DIST_ADJUST);
                } else if (distance > 36) {
                    move (0, SPEED, 0);
                    state = State.GET_DISTANCE;
                    nextStates.push(State.BOTTOM_FIELD_DIST_ADJUST);
                } else {
                    move(0, 0, 0);
                    state = State.DELAY;
                    delay = 1;
                    nextStates.push(State.BOTTOM_FIELD_ADJUST_2);
                }
                break;
            case BOTTOM_FIELD_ADJUST_2:
                if (distance < 33) {
                    move (0, -SPEED, 0);
                    state = State.GET_DISTANCE;
                    nextStates.push(State.BOTTOM_FIELD_DIST_ADJUST);
                } else if (distance > 36) {
                    move (0, SPEED, 0);
                    state = State.GET_DISTANCE;
                    nextStates.push(State.BOTTOM_FIELD_DIST_ADJUST);
                } else {
                    move(0, 0, 0);
                    state = State.DELAY;
                    delay = 1;
                    nextStates.push(State.BOTTOM_FIELD_STRAIGHTEN);
                }
                break;
            case BOTTOM_FIELD_STRAIGHTEN:
                if (heading < -5) {
                    move(0, 0, -.2);
                } else if (heading > 5) {
                    move(0, 0, .2);
                } else {
                    move(0, 0, 0);
                    state = State.DELAY;
                    delay = 1;
                    nextStates.push(State.START_COLUMN_COUNTING);
                }
                break;
            case START_COLUMN_COUNTING:
                move(0, 0, 0);
                objectCount = 0;
                USpivot.setPosition(.35);
                state = State.DELAY;
                delay = 2;
                nextStates.push(State.COLUMN_COUNTING);
                nextStates.push(State.GET_DISTANCE);
                prevdistance = 0;
                distanceFilter.clear();
                while (distanceFilter.size() < FILTER_BUFFER_LENGTH)
                    distanceFilter.add(255);
                break;
            case COLUMN_COUNTING:
                USpivot.setPosition(.35);
                int curdistance = (int) rangeSensor.getDistance(DistanceUnit.CM);
                if (curdistance == 0 || curdistance > 100)
                    break;
                distanceFilter.add(curdistance);
                distanceFilter.remove();
                double bufferTotal = 0;
                for (int dist : distanceFilter)
                    bufferTotal += dist;
                distance = bufferTotal / FILTER_BUFFER_LENGTH;
                // Oriented with sensor facing Cryptobox
                if (RED_TEAM) {
                    move(-SPEED, SPEED * Math.tan(Math.toRadians(7)), 0);
                } else {
                    move(SPEED, SPEED * Math.tan(Math.toRadians(7)), 0);
                }
                readings.push((int) distance);
                if (distance - prevdistance <= 1 && prevdistance != 0) {
                    if (prevdistance > distBeforeIncrease + 4 && distBeforeIncrease != 0)
                        objectCount++;
                    distBeforeIncrease = distance;
                }
                prevdistance = distance;


                if (keyColumn == objectCount) {
                    /*if (RED_TEAM) {
                        move(SPEED, 0, 0);
                    } else {
                        move(-SPEED, 0, 0);
                    }
                    */
                    move(0, 0, 0);
                    delay = 1;
                    nextStates.push(State.PUSH_GLYPH_IN_ADJUST);
                    state = State.DELAY;
                    break;
                }
                //state = State.GET_DISTANCE;
                //nextStates.push(State.COLUMN_COUNTING);
                times.push((int) (1000 * time));
                lastTime = time;
                break;
            case PUSH_GLYPH_IN_ADJUST:
                if (heading < -20) {
                    move(0, 0, -.2);
                } else if (heading > -13 + (keyColumn == 3 ? 3 : 0)) {
                    move(0, 0, .2);
                } else {
                    move(0, 0, 0);
                    state = State.DELAY;
                    delay = 1;
                    nextStates.push(State.PUSH_GLYPH_IN_LOWER);
                }
                break;
            case PUSH_GLYPH_IN_LOWER:
                delay = 2;
                lifter.setPower(-.5);
                state = State.DELAY;
                nextStates.push(State.PUSH_GLYPH_IN);
                break;
            case PUSH_GLYPH_IN:
                // Once key column has been found, the robot oscillates between PUSH_GLYPH_IN and PUSH_GLYPH_IN_ADJUST to ensure glyph gets into key column.
                move(0, SPEED, 0);
                lifter.setPower(0);

                delay = 1.2;
                nextStates.push(State.PUSH_GLYPH_IN_RELEASE);
                state = State.DELAY;
                break;
            case PUSH_GLYPH_IN_RELEASE:
                move(0, 0, 0);
                s1.setPosition(.9);
                s2.setPosition(.1);
                delay = .8;
                state = State.DELAY;
                nextStates.push(State.BACK_OFF_GLYPH);
                break;
            case BACK_OFF_GLYPH:
                move(0, -SPEED, 0);
                delay = 2;
                state = State.DELAY;
                nextStates.push(State.STOP);
                break;
            case DELAY:
                delayStart = time;
                state = State.DELAY_LOOP;
                break;
            case DELAY_LOOP:
                telemetry.addData("delaying", nextStates.peek());
                if (time > delayStart + delay)
                    state = nextStates.pop();
                break;
            case STOP:
                telemetry.addData("readings", readings);
                telemetry.addData("timings", times);
                move(0, 0, 0);
                break;
            case GET_DISTANCE:
                distance = 255;
                readingDistNum = 2;
                state=State.GET_DISTANCE_LOOP;
                break;
            case GET_DISTANCE_LOOP:
                double currentdist= rangeSensor.getDistance(DistanceUnit.CM);
                if(currentdist >200 || currentdist==0){
                    return;
                }
                readingDistNum--;
                if(currentdist < distance ) {
                    distance = currentdist;
                }
                if (readingDistNum==0){
                  state=nextStates.pop();
                }
                break;
        }

        telemetry.addData("State", state);
        telemetry.addData("Current distance is ", distance);
        telemetry.addData("Objects passed: ", objectCount);
        telemetry.addData("Key column", keyColumn);
    }

    // double x = gamepad1.right_stick_x;
    // double y = gamepad1.right_stick_y;
    // double dir = gamepad1.right_trigger or gamepad1.left_trigger;
    public void move(double x, double y, double dir) {
        if (dir != 0) {
            motor1.setPower(dir);
            motor2.setPower(dir);
            motor3.setPower(-dir);
            motor4.setPower(-dir);
        } else {
            double n = ((x + y) / Math.sqrt(2.0)); // n is the power of the motors in the +x +y direction
            double m = ((x - y) / Math.sqrt(2.0)); // m is the power of the motors in the +x -y direction
            motor1.setPower(m);
            motor2.setPower(n);
            motor3.setPower(m);
            motor4.setPower(n);
        }
    }
}