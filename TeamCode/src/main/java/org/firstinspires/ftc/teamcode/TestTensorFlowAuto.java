package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.util.List;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

@Autonomous
public class TestTensorFlowAuto<tfod> extends OpMode {

    private static final String TFOD_MODEL_ASSET = "FreightFrenzy_BCDM.tflite";
    private static final String[] LABELS = {
            "Ball",
            "Cube",
            "Duck",
            "Marker"
    };

    private static final String VUFORIA_KEY =
            "ASIwwWv/////AAABmR/+9d4sSkVEshzTIOkfUgAWTcQCqWQ3NeZFwrYj+HewIITQOcdzK95pLGiq3w+muSW12YMucPY4gr+LXUWae13of2pAVIwC03KapsTkznFaL5vJQvBSmir72Q0XFzO975UhES7phEj54qmV0HANvVXc9SVvzljLiSJvJt/6eDUEyqco/rUOnneZhEarLqZch8ma+TNUbWnNO4HnNu+E31xQVjR1ADGmSpln14EFvrLD22aWyGRFufLDPxMNZ0+HYMQg2rmyDK1HFxDnk6qpvtCYTjIXcLpUPXaDF5if3wIO3mDOaTk0OwdnBav9N1/bmwmYdEzjhRnTb7A8UCAnAUSxlAYIIH3WABg2FvfhQsRJ";

    private VuforiaLocalizer vuforia;
    public TFObjectDetector tfod;

    boolean Duck = false;
    boolean left = false;
    boolean right = false;
    boolean middle = false;


    public void scan() {

        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {

                telemetry.addData("# Object Detected", updatedRecognitions.size());

                // step through the list of recognitions and display boundary info.
                int i = 0;
                for (Recognition recognition : updatedRecognitions) {
                    telemetry.addData(String.format("label (%d)", i), recognition.getLabel());

                    telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                            recognition.getLeft(), recognition.getTop());
                    telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                            recognition.getRight(), recognition.getBottom());
                    i++;

                    if (recognition.getLabel().equals("Duck")) {
                        Duck = true;
                        telemetry.addData("Object Detected", "Duck");
                    } else {
                        Duck = false;
                        telemetry.addData("Object Detected", "None");
                    }
                    double duckPos = recognition.getRight();

                    if (Duck == true) {
                        if (duckPos > 500) {
                            telemetry.addData("posistion:", "right");
                            right = true;
                        } else if (duckPos < 500) {
                            telemetry.addData("posistion:", "middle");
                            middle = true;
                        } else{
                            right= false;
                            left = false;
                        }
                    }
                }
                if (right == false && middle == false) {
                    telemetry.addData("posistion:", "left");
                    left = true;
                }
                telemetry.update();
            }

        }
//        if (tfod != null) {
//            tfod.shutdown();
//        }
    }

    private void initVuforia() {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        vuforia = ClassFactory.getInstance().createVuforia(parameters);

    }

    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
    }

    @Override
    public void init() {
        initVuforia();
        initTfod();
    }

    @Override
    public void init_loop() {
        if (tfod != null) {
            tfod.activate();
            tfod.setZoom(1, 16.0 / 9.0);
        }
    }

    @Override
    public void stop() {
        if (tfod != null) {
            tfod.shutdown();
        }
    }

    @Override
    public void loop() {
        scan();
        telemetry.update();
    }
}