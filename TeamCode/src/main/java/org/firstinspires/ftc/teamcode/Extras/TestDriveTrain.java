package org.firstinspires.ftc.teamcode.Extras;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

//@Disabled
@Autonomous
public class TestDriveTrain extends OpMode {

    // initializing the IMU
    BNO055IMU imu;
    Orientation angles;

    // initializing the motors
    private DcMotor RFMotor;
    private DcMotor LFMotor;
    private DcMotor RBMotor;
    private DcMotor LBMotor;
    private Servo Arm;
    private Servo ArmR;

    double RFPreviousValue = 0;
    double RBPreviousValue = 0;
    double LFPreviousValue = 0;
    double LBPreviousValue = 0;

    // one rotaion
    double one = 537.6;
    ElapsedTime t1 = new ElapsedTime();

    double avgEnc;
    double avgEncSide;

    // finds the robot heading (degrees)
    double getHeading() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        this.imu.getPosition();
        return angles.firstAngle;
    }

    double encoderSpeed(double targetPosition, double maxSpeed) {
        double AverageEncoderPosition = (RFMotor.getCurrentPosition() - RFPreviousValue + LFMotor.getCurrentPosition() - LFPreviousValue + RBMotor.getCurrentPosition() - RBPreviousValue + LBMotor.getCurrentPosition() - LBPreviousValue) / 4;
        double distance = targetPosition - AverageEncoderPosition;
        //telemetry.addData("Encoder Speed distance",distance);
        double speed = Range.clip(distance / 500, -maxSpeed, maxSpeed); // clip the speed
        return speed;
    }

    private double encoderSpeedSide(double targetPosition, double maxSpeed) {
        double avgEncPosition = (-(LFMotor.getCurrentPosition() - LFPreviousValue) - (RBMotor.getCurrentPosition() - RBPreviousValue) + (RFMotor.getCurrentPosition() - RFPreviousValue) + (LBMotor.getCurrentPosition() - LBPreviousValue)) / 4;
        double distance = targetPosition - avgEncPosition;
        telemetry.addData("difference", distance);
        double power = Range.clip(  -distance / 500, -maxSpeed, maxSpeed);
        return power;
    }

    public void setForwardPower(double turnPower, double power) {
        RFMotor.setPower(turnPower - power);
        LFMotor.setPower(turnPower - power);
        RBMotor.setPower(turnPower - power);
        LBMotor.setPower(turnPower - power);
        telemetry.addData("forward power", turnPower);
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

    double turn(double targetAngle) {
        getHeading();
        double turnAngle = targetAngle - getHeading();
        // telemetry.addData("turnAngle", turnAngle);
        double power = Range.clip(turnAngle / 50, -0.3, 0.3);
        return power;
    }

    public void rampUp(double distance, double heading, double time, double maxSpeed) {
        double AvgEncPos = (RFMotor.getCurrentPosition() - RFPreviousValue + (-LFMotor.getCurrentPosition()) - LFPreviousValue + RBMotor.getCurrentPosition() - RBPreviousValue + LBMotor.getCurrentPosition() - LBPreviousValue) / 4;
        double AccelerationSlope = maxSpeed / time;
        double power = t1.seconds() * AccelerationSlope;
        if (Math.abs(power) < Math.abs(encoderSpeed(distance, maxSpeed))) { // if acceleration is less than speed
            setForwardPower(turn(heading), power);  //then set motor power to turn towards heading and accelerate until max speed
        } else {
            if (!(Math.abs(distance - AvgEncPos) < 80)) {
                telemetry.addData("motor is: ", "busy");
                setForwardPower(turn(heading), encoderSpeed(distance, maxSpeed));// otherwise keep motor power to heading and stop at the target Encoder Position
            } else {
//                  rampUp(0,0,0,0);
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
            if (!(Math.abs(distance - AvgEncPos) < 100) && (Math.abs(heading - getHeading()) < 5)) {
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
        double AvgEncPos = (RFMotor.getCurrentPosition() + (-LFMotor.getCurrentPosition()) + RBMotor.getCurrentPosition() + LBMotor.getCurrentPosition()) / 4;
        double AccelerationSlope = maxSpeed / time;
        double power = t1.seconds() * AccelerationSlope;
        if (Math.abs(power) < Math.abs(encoderSpeed(distance, maxSpeed))) { // if acceleration is less than speed
            setTurnPower(turn(heading), power);  //then set motor power to turn towards heading and accelerate until max speed
        } else {
            if (!(Math.abs(heading - getHeading()) < 15)) {
                telemetry.addData("motor is: ", "busy");
                setTurnPower(turn(heading), encoderSpeed(distance, maxSpeed));// otherwise keep motor power to heading and stop at the target Encoder Position
            } else {
                telemetry.addData("motor is: ", "not busy");
                RFMotor.setPower(0);
                LFMotor.setPower(0);
                RBMotor.setPower(0);
                LBMotor.setPower(0);
                setTurnPower(0, 0);

            }
        }
    }


    @Override
    public void init() {
        telemetry.addData(">", "Press Play to start op mode");

        LFMotor = hardwareMap.get(DcMotor.class, "LFMotor");
        RFMotor = hardwareMap.get(DcMotor.class, "RFMotor");
        LBMotor = hardwareMap.get(DcMotor.class, "LBMotor");
        RBMotor = hardwareMap.get(DcMotor.class, "RBMotor");
        Arm = hardwareMap.get(Servo.class, "Arm");
        ArmR = hardwareMap.get(Servo.class, "ArmR");
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        LFMotor.setDirection(DcMotor.Direction.REVERSE);
        RFMotor.setDirection(DcMotor.Direction.FORWARD);
        LBMotor.setDirection(DcMotor.Direction.REVERSE);
        RBMotor.setDirection(DcMotor.Direction.FORWARD);

        telemetry.addData("Status", "Initialized");
        RFMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LFMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LFMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RFMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LBMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RBMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Gyro stuff
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.loggingEnabled=false;
        imu.initialize(parameters);
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
    }


    @Override
    public void loop() {
        if(Arm.getPosition() != 0.9032) {
            Arm.setPosition(0.9032);
            ArmR.setPosition(0.11);
        }

        telemetry.addData("Heading", getHeading());
        telemetry.addData("LF Distance", LFMotor.getCurrentPosition());
        telemetry.addData("RF Distance", RFMotor.getCurrentPosition());
        telemetry.addData("LB Distance", LBMotor.getCurrentPosition());
        telemetry.addData("RB Distance", RBMotor.getCurrentPosition());
    }
}
