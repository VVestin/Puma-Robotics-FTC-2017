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

    //Servo
    private Servo USpivot;

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
    private double distance=255;

    private double prevdistance;
    private List<Integer> readings;

    private int readingDistNum;


    public static final double SPEED = 0.2;

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

        rangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range");

        orientationSensor = new OrientationSensor(hardwareMap);

        state = State.START;
        nextStates = new Stack<State>();

        vuforia = new VuforiaHelper2(hardwareMap);

        USpivot= hardwareMap.servo.get("usp");
        USpivot.setPosition(1);

        readings=new LinkedList<Integer>();

    }

    public void start() {
        vuforia.start();

        USpivot.setPosition(0.5);
    }

    public void loop() {
        vuforia.loop();
        if (vuforia.getKeyColumn() != 0) {
            keyColumn = vuforia.getKeyColumn();
        }

        double heading = orientationSensor.getOrientation();

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
                move(0, SPEED, 0);
                if (distance < 38) {
                    move(0, 0, 0);
                    state = State.START_COLUMN_COUNTING;
                }
                else {
                    state = State.GET_DISTANCE;
                    state = nextStates.push(State.BOTTOM_FIELD);
                }
                break;
            case START_COLUMN_COUNTING:
                walldistance += distance;
                numReadings++;
                if (numReadings == 5) {
                    walldistance /= numReadings;
                    telemetry.addData("Wall Distance", walldistance);
                    state = State.DELAY;
                    USpivot.setPosition(0.35);
                    nextStates.push(State.COLUMN_COUNTING);
                }
                break;
            case COLUMN_COUNTING:
                // Oriented with sensor facing Cryptobox
                USpivot.setPosition(0.35); // TODO Add a delay
                delay= 0.5;
                if (RED_TEAM) {
                    move(-SPEED, 0, 0);
                } else {
                    move(SPEED, 0, 0);
                }
                readings.add((int) distance);
                telemetry.addData("distance", rangeSensor.getDistance(DistanceUnit.CM));
                if (distance >= prevdistance + 5) {
                    objectCount += 1;
                }
                prevdistance = distance;

                telemetry.addData("Current distance is ", distance);
                telemetry.addData("Objects passed: ", objectCount);
                if (keyColumn == objectCount) {
                    telemetry.addData("Readings:", readings);
                    move(0, 0, 0);
                    delay = 0.7;
                    nextStates.push(State.PUSH_GLYPH_IN);
                    state = State.DELAY;
                    break;
                }
                state = State.GET_DISTANCE;
                nextStates.push(State.COLUMN_COUNTING);
                break;
            case PUSH_GLYPH_IN:
                move(0, SPEED, 0);
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
            case GET_DISTANCE:
                distance=rangeSensor.getDistance(DistanceUnit.CM);
                readingDistNum=2;
                state=State.GET_DISTANCE_LOOP;
                break;
            case GET_DISTANCE_LOOP:
                double currentdist= rangeSensor.getDistance(DistanceUnit.CM);
                if(currentdist >200 || currentdist==0){
                    return;
                }
                readingDistNum--;
                if(currentdist < distance ) {
                distance=currentdist;
                }
                if (readingDistNum==0){
                  state=nextStates.pop();
                }
                break;
        }

        telemetry.addData("State", state);
    }

    // double x = gamepad1.right_stick_x;
    // double y = gamepad1.right_stick_y;
    // double dir = gamepad1.right_trigger or gamepad1.left_trigger;
    public void move(double x, double y, double dir) {
        double n = ((x + y) / Math.sqrt(2.0)); // n is the power of the motors in the +x +y direction
        double m = ((x - y) / Math.sqrt(2.0)); // m is the power of the motors in the +x -y direction
        motor1.setPower(m);
        motor2.setPower(n);
        motor3.setPower(m);
        motor4.setPower(n);

        telemetry.addData("m: ", m);
        telemetry.addData("n: ", n);
    }
}