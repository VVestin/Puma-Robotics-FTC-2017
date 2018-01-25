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
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * Created by vvestin on 11/11/17.
 */

public class VuforiaHelper implements GameConstants {
    public static final String TAG = "Vuforia VuMark Sample";
    public static final String VUFORIA_KEY = "AdrNx7L/////AAAAGWscSRJwJ0For7OwbugY3Fd0I3f+mAuH+BAHpz7UBuNJnU+QudRFM8gzxBh+mZcuiwi2TStZTxHuDQvVJHER5zuUmh7X6dr/7uEnPy+OBd72HjBc2gM+w7DNmcBhY8SmEgLRlzhI4dRCAmjADeVQd9c/vTTyqWSYdy7F2fE2eQbSoXKyKN1uFV6P6lN3NlHSazLOaniTLpAQQlbOwb9S2KxXy7PQK1ZBAmWMdHb5jwAXaqz+HXMPBez6/7behYzk1eu4a/0hFZ6jWo9Khoc9MRrhmCac0SCzmNRjfD8Y9Q61EtWvmo+WlbyzFJUNsZbND80BXAKaOWXvCAsdCo58qGtmVr36Bau5iljOe5HBbvov";

    VuforiaLocalizer vuforia;
    VuforiaTrackable relicTemplate;
    VuforiaTrackables relicTrackables;
    RelicRecoveryVuMark vuMark;

    public void start() {
        relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = relicTrackables.get(0);

        relicTrackables.activate();
    }

    public VuforiaHelper (HardwareMap hardwareMap) {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
    }

    public void loop () {
        vuMark = RelicRecoveryVuMark.from(relicTemplate);
    }

    public int getKeyColumn () {
        if (vuMark == RelicRecoveryVuMark.LEFT) {
            return RED_TEAM ? 3 : 1;
        }
        if (vuMark == RelicRecoveryVuMark.RIGHT) {
            return RED_TEAM ? 1 : 3;
        }
        if (vuMark == RelicRecoveryVuMark.CENTER) {
            return 2;
        }
        else {
            return 0;
        }
    }


    String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }
}
