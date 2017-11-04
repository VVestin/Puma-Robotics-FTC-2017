package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * Created by vvestin on 10/21/17.
 */
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Autonomous", group = "Auto")

public class Autonomous extends OpMode {

    public static final String TAG = "Vuforia VuMark Sample";
    public static final String VUFORIA_KEY = "AdrNx7L/////AAAAGWscSRJwJ0For7OwbugY3Fd0I3f+mAuH+BAHpz7UBuNJnU+QudRFM8gzxBh+mZcuiwi2TStZTxHuDQvVJHER5zuUmh7X6dr/7uEnPy+OBd72HjBc2gM+w7DNmcBhY8SmEgLRlzhI4dRCAmjADeVQd9c/vTTyqWSYdy7F2fE2eQbSoXKyKN1uFV6P6lN3NlHSazLOaniTLpAQQlbOwb9S2KxXy7PQK1ZBAmWMdHb5jwAXaqz+HXMPBez6/7behYzk1eu4a/0hFZ6jWo9Khoc9MRrhmCac0SCzmNRjfD8Y9Q61EtWvmo+WlbyzFJUNsZbND80BXAKaOWXvCAsdCo58qGtmVr36Bau5iljOe5HBbvov";
    OpenGLMatrix lastLocation = null;

    VuforiaLocalizer vuforia;
    VuforiaTrackable relicTemplate;
    VuforiaTrackables relicTrackables;
    RelicRecoveryVuMark vuMark;

    private DcMotor motor1;
    private DcMotor motor2;
    private DcMotor motor3;
    private DcMotor motor4;

    private State state;

    private int keyColumn = 1;
    int objectCount;
    boolean seeColumn = false;

    //Changes
    boolean redTeam = true;
    boolean topField = false;

    ModernRoboticsI2cRangeSensor rangeSensor;

    private OrientationSensor orientationSensor;
    private OrientationSensor orientationSensor2;

    public void start() {
        relicTrackables.activate();
    }

    public void init() {
        motor1 = hardwareMap.dcMotor.get("w1");
        motor2 = hardwareMap.dcMotor.get("w2");
        motor3 = hardwareMap.dcMotor.get("w3");
        motor4 = hardwareMap.dcMotor.get("w4");

        motor1.setDirection(DcMotorSimple.Direction.REVERSE);
        motor2.setDirection(DcMotorSimple.Direction.REVERSE);

        rangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "sensor_range");

        orientationSensor = new OrientationSensor(hardwareMap);
        orientationSensor2 = new OrientationSensor(hardwareMap);

        state = State.START;

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        parameters.vuforiaLicenseKey=VUFORIA_KEY;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate");

        telemetry.addData(">", "Press Play to start");
        telemetry.update();
    }

    public void loop() {

            /**
             * See if any of the instances of {@link relicTemplate} are currently visible.
             * {@link RelicRecoveryVuMark} is an enum which can have the following values:
             * UNKNOWN, LEFT, CENTER, and RIGHT. When a VuMark is visible, something other than
             * UNKNOWN will be returned by {@link RelicRecoveryVuMark#from(VuforiaTrackable)}.
             */
            vuMark = RelicRecoveryVuMark.from(relicTemplate);
            if (vuMark != RelicRecoveryVuMark.UNKNOWN) {

                telemetry.addData("VuMark", "%s visible", vuMark);

                OpenGLMatrix pose = ((VuforiaTrackableDefaultListener)relicTemplate.getListener()).getPose();
                telemetry.addData("Pose", format(pose));

                if (pose != null) {
                    VectorF trans = pose.getTranslation();
                    Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

                    double tX = trans.get(0);
                    double tY = trans.get(1);
                    double tZ = trans.get(2);

                    double rX = rot.firstAngle;
                    double rY = rot.secondAngle;
                    double rZ = rot.thirdAngle;
                }
            else {
                telemetry.addData("VuMark", "not visible");
            }

            telemetry.update();
        }

        double heading = orientationSensor.getOrientation();
        double heading2 = orientationSensor.getOrientation();

        double distance = rangeSensor.getDistance(DistanceUnit.CM);

        switch (state) {

            case START:
                if (topField == true) {
                    state = State.TOP_FIELD;
                }
                else if (topField == false) {
                    state = State.BOTTOM_FIELD;
                }
                break;
            case COLUMN_COUNTING:
                //Oriented with sensor facing Cryptobox
                if (redTeam == true) {
                    move(-1,0,0);
                }
                if (redTeam == false) {
                    move(1,0,0);
                }
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
                if(keyColumn == 1){
                    if (objectCount == 1) {
                        state=State.STOP;
                    }
                    else if(redTeam==true){
                        move(1,0,0);
                    }
                    else {
                        move(-1,0,0);
                    }
                }
                if(keyColumn == 2){
                    if (objectCount == 2) {
                        state=State.STOP;
                    }
                    else if(redTeam==true){
                        move(1,0,0);
                    }
                    else {
                        move (-1,0,0);
                    }
                }
                if(keyColumn == 3){
                    if (objectCount == 3) {
                        state=State.STOP;
                    }
                    else if(redTeam==true){
                        move(1,0,0);
                    }
                    else {
                        move (-1,0,0);
                    }
                }
                telemetry.addData("Current distance is ", distance);
                telemetry.addData("Objects passed: ", objectCount);
                if(keyColumn==objectCount){
                    state=State.STOP;
                }
                break;
            case STOP:
               move(0, 0, 0);
                break;
            case TOP_FIELD:
                    if (redTeam == true) {
                        if (vuMark == RelicRecoveryVuMark.CENTER || vuMark == RelicRecoveryVuMark.LEFT || vuMark == RelicRecoveryVuMark.RIGHT) {
                            move(-1,0,0);
                        if (distance < 20) {
                            move(0, 0, 0);
                            state = State.COLUMN_COUNTING;
                        }
                        }
                    } else if (redTeam == false) {
                        if (vuMark == RelicRecoveryVuMark.CENTER || vuMark == RelicRecoveryVuMark.LEFT || vuMark == RelicRecoveryVuMark.RIGHT) {
                            move(1,0,0);
                            if (distance < 20) {
                                move(0, 0, 0);
                                state = State.COLUMN_COUNTING;
                            }
                        }
                    }
                break;
            case BOTTOM_FIELD:
                if (vuMark == RelicRecoveryVuMark.CENTER || vuMark == RelicRecoveryVuMark.LEFT || vuMark == RelicRecoveryVuMark.RIGHT) {
                    move(1, 0, 0);
                    if (distance < 20) {
                        move(0, 0, 0);
                        state = State.COLUMN_COUNTING;
                    }
                }
                break;
        }
        telemetry.addData("heading is: ", heading);
        telemetry.addData("second heading is: ", heading2);
    }

    String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }

    public void move(double x,double y,double direction){
        direction=motor3.getCurrentPosition() + motor4.getCurrentPosition() - motor1.getCurrentPosition() - motor2.getCurrentPosition();
        x=motor1.getCurrentPosition() + motor2.getCurrentPosition() + motor3.getCurrentPosition() + motor4.getCurrentPosition();
        y=motor1.getCurrentPosition() + motor2.getCurrentPosition() - motor3.getCurrentPosition() + motor4.getCurrentPosition();
    }
}