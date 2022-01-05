package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

@Autonomous
public class FreightFrenzyAutoREDR extends OpMode {
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

    BNO055IMU imu;
    Orientation angles;
    private DcMotor RFMotor;
    private DcMotor LFMotor;
    private DcMotor RBMotor;
    private DcMotor LBMotor;

    double RFPreviousValue = 0;
    double RBPreviousValue = 0;
    double LFPreviousValue = 0;
    double LBPreviousValue = 0;

    double one = 537.6;

    ElapsedTime t1 = new ElapsedTime();
    ElapsedTime runtime = new ElapsedTime();

    double AverageEncoderPosition;

    double encoderSpeed(double targetPosition, double maxSpeed) {
        AverageEncoderPosition = (RFMotor.getCurrentPosition() - RFPreviousValue + LFMotor.getCurrentPosition() - LFPreviousValue + RBMotor.getCurrentPosition() - RBPreviousValue + LBMotor.getCurrentPosition() - LBPreviousValue) / 4;
        double distance = targetPosition - AverageEncoderPosition;
        double speed = Range.clip(-distance / 500, -maxSpeed, maxSpeed);
        return speed;
    }

    private double encoderSpeedSide(double targetPosition, double maxSpeed) {
        double avgEncPosition = (-(LFMotor.getCurrentPosition() - LFPreviousValue) - (RBMotor.getCurrentPosition() - RBPreviousValue) + (RFMotor.getCurrentPosition() - RFPreviousValue) + (LBMotor.getCurrentPosition() - LBPreviousValue)) / 4;
        double distance = targetPosition - avgEncPosition;
        telemetry.addData("difference", distance);
        double power = Range.clip(distance / 500, -maxSpeed, maxSpeed);
        return power;
    }

    public void setForwardPower(double turnPower, double power) {
        RFMotor.setPower(turnPower - power);
        LFMotor.setPower(turnPower - power);
        RBMotor.setPower(turnPower - power);
        LBMotor.setPower(turnPower - power);
        telemetry.addData("turn power", turnPower);
    }

    public void setTurnPower(double turnPower, double power) {
        RFMotor.setPower(-turnPower - power);
        LFMotor.setPower(turnPower - power);
        RBMotor.setPower(-turnPower - power);
        LBMotor.setPower(turnPower - power);
        telemetry.addData("turn power", turnPower);
    }

    private void driveSideways(double turnPower, double encoderSpeedSide) {
        RFMotor.setPower(turnPower + encoderSpeedSide);
        LFMotor.setPower(-turnPower - encoderSpeedSide);
        RBMotor.setPower(-turnPower - encoderSpeedSide);
        LBMotor.setPower(turnPower + encoderSpeedSide);
    }

    double getHeading() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        this.imu.getPosition();
        return angles.firstAngle;
    }

    double turn(double targetAngle) {
        getHeading();
        double turnAngle = targetAngle - getHeading();
        // telemetry.addData("turnAngle", turnAngle);
        double power = Range.clip(turnAngle / 50, -0.3, 0.3);
        return power;
    }

    public void rampUp(double distance, double heading, double time, double maxSpeed) {
        double AvgEncPos = (RFMotor.getCurrentPosition() - RFPreviousValue + LFMotor.getCurrentPosition() - LFPreviousValue + RBMotor.getCurrentPosition() - RBPreviousValue + LBMotor.getCurrentPosition() - LBPreviousValue) / 4;
        double AccelerationSlope = maxSpeed / time;
        double power = t1.seconds() * AccelerationSlope;
        if (Math.abs(power) < Math.abs(encoderSpeed(distance, maxSpeed))) { // if acceleration is less than speed
            setForwardPower(turn(heading), power);  //then set motor power to turn towards heading and accelerate until max speed
        } else {
            if (!(Math.abs(distance - AvgEncPos) < 80)) {
                telemetry.addData("motor is: ", "busy");
                setForwardPower(turn(heading), encoderSpeed(distance, maxSpeed));// otherwise keep motor power to heading and stop at the target Encoder Position
            } else {
                telemetry.addData("motor is: ", "not busy");
                RFMotor.setPower(0);
                LFMotor.setPower(0);
                RBMotor.setPower(0);
                LBMotor.setPower(0);
                setForwardPower(0, 0);
            }
        }
    }

    boolean tripLoopDoneSide = false;

    private void rampUpSide(double distance, double heading, double time, double maxSpeed) {
        double AvgEncPos = (-(LFMotor.getCurrentPosition() - LFPreviousValue) - (RBMotor.getCurrentPosition() - RBPreviousValue) + (RFMotor.getCurrentPosition() - RFPreviousValue) + (LBMotor.getCurrentPosition() - LBPreviousValue)) / 4;
        telemetry.addData("Average Encoder Posistion Sideways: ", AvgEncPos);
        double AccelerationSlope = maxSpeed / time;
        double power = t1.seconds() * AccelerationSlope;
        if (Math.abs(power) < Math.abs(encoderSpeedSide(distance, maxSpeed))) {
            driveSideways(turn(heading), power);
        } else {
            if (!(Math.abs(distance - AvgEncPos) < 100)) {
                telemetry.addData("motor is: ", "busy");
                driveSideways(turn(heading), encoderSpeedSide(distance, maxSpeed));// otherwise keep motor power to heading and stop at the target Encoder Position
            } else {
                telemetry.addData("motor is: ", "not busy");
                RFMotor.setPower(0);
                LFMotor.setPower(0);
                RBMotor.setPower(0);
                LBMotor.setPower(0);
                setTurnPower(0, 0);
                tripLoopDoneSide = true;
            }
        }
    }

    public void rampUpTurn(double distance, double heading, double time, double maxSpeed) {
        double AvgEncPos = (RFMotor.getCurrentPosition() + LFMotor.getCurrentPosition() + RBMotor.getCurrentPosition() + LBMotor.getCurrentPosition()) / 4;
        double AccelerationSlope = maxSpeed / time;
        double power = t1.seconds() * AccelerationSlope;
        if (Math.abs(power) < Math.abs(encoderSpeed(distance, maxSpeed))) {
            setTurnPower(turn(heading), power);
        } else {
            if (!(Math.abs(heading - getHeading()) < 15)) {
                telemetry.addData("motor is: ", "busy");
                setTurnPower(turn(heading), encoderSpeed(distance, maxSpeed));
            } else {
                telemetry.addData("motor is: ", "not busy");
                telemetry.addData("heading:", getHeading());
                RFMotor.setPower(0);
                LFMotor.setPower(0);
                RBMotor.setPower(0);
                LBMotor.setPower(0);
                setTurnPower(0, 0);
            }
        }
    }

    public void sahithLetsGetLitletsGoooooo(double distance, double heading, double time, double maxSpeed) {
        double AccelerationSlope = maxSpeed / time;
        double power = t1.seconds() * AccelerationSlope;
        if (Math.abs(power) < Math.abs(encoderSpeedSide(distance, maxSpeed))) {
            driveSideways(turn(heading), power);
        } else {
            telemetry.addData("motor is: ", "busy");
            driveSideways(turn(heading), encoderSpeedSide(distance, maxSpeed));// otherwise keep motor power to heading and stop at the target Encoder Position
        }
    }

    boolean tripLoopDone = false;
    boolean EncoderPower;

    boolean tripLoop() {
        double AverageEncPower = (RFMotor.getPower() + LFMotor.getPower() + RBMotor.getPower() + LBMotor.getPower()) / 4;

        if (AverageEncPower == 0) {
            EncoderPower = false;
        } else {
            EncoderPower = true;
        }
        if (!tripLoopDone && EncoderPower) {
            tripLoopDone = true;
        }
        if (tripLoopDone && !EncoderPower || tripLoopDoneSide) {
            RFPreviousValue = RFMotor.getCurrentPosition();
            RBPreviousValue = RBMotor.getCurrentPosition();
            LFPreviousValue = LFMotor.getCurrentPosition();
            LBPreviousValue = LBMotor.getCurrentPosition();
            tripLoopDoneSide = false;
            telemetry.addData("tripLoop return:", "TRUE");
            return true;
        } else {
            telemetry.addData("tripLoop return:", "FALSE");
            return false;
        }
    }

    boolean tripLoopSideways() {
        if (tripLoopDoneSide) {
            RFPreviousValue = RFMotor.getCurrentPosition();
            RBPreviousValue = RBMotor.getCurrentPosition();
            LFPreviousValue = RFMotor.getCurrentPosition();
            LBPreviousValue = RFMotor.getCurrentPosition();
            tripLoopDoneSide = false;
            return true;
        }
        return false;
    }
    double duckPos;
    public void scan() {

        if (tfod != null) {
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {

                telemetry.addData("# Object Detected", updatedRecognitions.size());
                
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
                    duckPos = recognition.getRight();

                    if (Duck == true) {
                        if (duckPos < 500) {
                            left = true;
                            telemetry.addData("position:", "left");
                        } else if (duckPos > 500) {
                            middle = true;
                            telemetry.addData("position:", "middle");
                        } else {
                            middle = false;
                            left = false;
                        }
                    }
                }
                if (middle == false && left == false) {
                    right = true;
                    telemetry.addData("position", "right");
                }
                telemetry.update();
            }
        }
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

    boolean trip1 = false;
    boolean trip2 = false;
    boolean trip3 = false;
    boolean trip4 = false;
    boolean trip5 = false;
    boolean trip6 = false;
    boolean trip7 = false;
    boolean trip8 = false;
    boolean trip9 = false;
    boolean trip10 = false;
    boolean trip11 = false;
    boolean trip12 = false;
    boolean trip13 = false;
    boolean trip14 = false;
    boolean trip15 = false;
    boolean trip16 = false;
    boolean trip17 = false;
    boolean trip18 = false;
    boolean trip19 = false;
    boolean trip20 = false;
    boolean trip21 = false;
    boolean trip22 = false;
    boolean trip23 = false;
    boolean trip24 = false;
    boolean trip25 = false;
    boolean trip26 = false;
    boolean trip27 = false;
    boolean trip28 = false;
    boolean trip29 = false;
    boolean trip30 = false;

    public void runAuto1() {
        if (left == true) {

        }
    }

    public void runAuto2() {
        if (middle == true) {
            if(!trip1) {
                //get out da wayy of da wall so i cnat turn
                rampUpSide(-one, 0,0,0.3);
                trip1 = tripLoopSideways();
                telemetry.addData("trip1",trip1);
            }
            //turn towards the hub
            else if(trip1 && !trip2){
                rampUpTurn(0,135,0.5,0.2);
                trip2 = tripLoop();
                telemetry.addData("trip2",trip2);
            }
            //go forward
            else if(trip2 && !trip3) {
                rampUp(-2 * one, 135, 0.5, 0.2);
                trip3 = tripLoop();
                telemetry.addData("trip3", trip3);
            }
        }
    }
    public void runAuto3() {
        if (right == true) {

        }
    }


    @Override
    public void init() {
        RFMotor = hardwareMap.get(DcMotor.class, "RFMotor");
        LFMotor = hardwareMap.get(DcMotor.class, "LFMotor");
        RBMotor = hardwareMap.get(DcMotor.class, "RBMotor");
        LBMotor = hardwareMap.get(DcMotor.class, "LBMotor");

        LFMotor.setDirection(DcMotor.Direction.REVERSE);
        RFMotor.setDirection(DcMotor.Direction.FORWARD);
        LBMotor.setDirection(DcMotor.Direction.REVERSE);
        RBMotor.setDirection(DcMotor.Direction.FORWARD);

        telemetry.addData("status", "initialized");

        RFMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LFMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LFMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RFMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LBMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RBMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        initVuforia();
        initTfod();
    }

    @Override
    public void init_loop() {
        telemetry.addData("LF Distance", LFMotor.getCurrentPosition());
        telemetry.addData("RF Distance", RFMotor.getCurrentPosition());
        telemetry.addData("LB Distance", LBMotor.getCurrentPosition());
        telemetry.addData("RB Distance", RBMotor.getCurrentPosition());

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        telemetry.addData("Heading: ", angles.firstAngle);
        telemetry.addData("Roll: ", angles.secondAngle);
        telemetry.addData("Pitch: ", angles.thirdAngle);
        telemetry.update();

        if (tfod != null) {
            tfod.activate();
            tfod.setZoom(1, 16.0 / 9.0);
        }

        if (Duck == true) {
            if (duckPos < 500) {
                left = true;
                telemetry.addData("position:", "left");
            } else if (duckPos > 500) {
                middle = true;
                telemetry.addData("position:", "middle");
            } else {
                middle = false;
                left = false;
            }
        }
        if (middle == false && left == false) {
            right = true;
            telemetry.addData("position", "right");
        }
    }

    @Override
    public void start() {
        t1.reset();
        runtime.reset();
    }

    @Override
    public void stop() {
//        if (tfod != null) {
//            tfod.shutdown();
//        }
    }

    @Override
    public void loop() {
        scan();
        runAuto1();
        runAuto2();
        runAuto3();

        telemetry.addData("heading:", getHeading());
        telemetry.addData("RFMotor encoder:", RFMotor.getCurrentPosition());
        telemetry.addData("LFMotor encoder:", LFMotor.getCurrentPosition());
        telemetry.addData("RBMotor encoder:", RBMotor.getCurrentPosition());
        telemetry.addData("LBMotor encoder:", LBMotor.getCurrentPosition());
        telemetry.update();
    }

    public final void idle() {
        Thread.yield();
    }

    public final void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

}

