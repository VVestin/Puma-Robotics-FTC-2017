package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * Created by vvestin on 11/11/17.
 */

public class VuforiaHelper {
    public static final String TAG = "Vuforia VuMark Sample";
    public static final String VUFORIA_KEY = "AdrNx7L/////AAAAGWscSRJwJ0For7OwbugY3Fd0I3f+mAuH+BAHpz7UBuNJnU+QudRFM8gzxBh+mZcuiwi2TStZTxHuDQvVJHER5zuUmh7X6dr/7uEnPy+OBd72HjBc2gM+w7DNmcBhY8SmEgLRlzhI4dRCAmjADeVQd9c/vTTyqWSYdy7F2fE2eQbSoXKyKN1uFV6P6lN3NlHSazLOaniTLpAQQlbOwb9S2KxXy7PQK1ZBAmWMdHb5jwAXaqz+HXMPBez6/7behYzk1eu4a/0hFZ6jWo9Khoc9MRrhmCac0SCzmNRjfD8Y9Q61EtWvmo+WlbyzFJUNsZbND80BXAKaOWXvCAsdCo58qGtmVr36Bau5iljOe5HBbvov";
    OpenGLMatrix lastLocation = null;

    VuforiaLocalizer vuforia;
    VuforiaTrackable relicTemplate;
    VuforiaTrackables relicTrackables;
    RelicRecoveryVuMark vuMark;

    public void start() {
        relicTrackables.activate();
    }

    public VuforiaHelper (HardwareMap hardwareMap) {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        parameters.vuforiaLicenseKey=VUFORIA_KEY;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate");
    }
    public void loop () {

        /**
         * See if any of the instances of {@link relicTemplate} are currently visible.
         * {@link RelicRecoveryVuMark} is an enum which can have the following values:
         * UNKNOWN, LEFT, CENTER, and RIGHT. When a VuMark is visible, something other than
         * UNKNOWN will be returned by {@link RelicRecoveryVuMark#from(VuforiaTrackable)}.
         */
        vuMark = RelicRecoveryVuMark.from(relicTemplate);
        if (vuMark != RelicRecoveryVuMark.UNKNOWN) {

            OpenGLMatrix pose = ((VuforiaTrackableDefaultListener)relicTemplate.getListener()).getPose();

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
            }

        }

    }

    public int getKeyColumn () {
        if (vuMark == RelicRecoveryVuMark.LEFT) {
            return 1;
        }
        if (vuMark == RelicRecoveryVuMark.RIGHT) {
            return 2;
        }
        if (vuMark == RelicRecoveryVuMark.CENTER) {
            return 3;
        }
        else {
            return 0;
        }
    }


    String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }
}
