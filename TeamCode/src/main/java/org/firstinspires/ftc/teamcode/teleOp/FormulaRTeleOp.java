package org.firstinspires.ftc.teamcode.teleOp;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.util.ArrayList;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp
public class FormulaRTeleOp extends OpMode {

    //initializing and variables
    BNO055IMU imu;
    Orientation angles;
    private DcMotor RFMotor;
    private DcMotor LFMotor;
    private DcMotor RBMotor;
    private DcMotor LBMotor;
    private DcMotor Intake;
    private Servo Arm;

    ElapsedTime t1 = new ElapsedTime();
    ElapsedTime t2 = new ElapsedTime();
    ElapsedTime t3 = new ElapsedTime();
    ElapsedTime t4 = new ElapsedTime();

    boolean Outake = false;

    int booleanIncrementer = 0;

    int buttonInc = 0;

    ArrayList<Boolean> booleanArray = new ArrayList<Boolean>();

    //DriveTrain method
    public void moveDriveTrain(){
        double vertical = 0;
        double horizontal = 0;
        double pivot = 0;
        vertical = gamepad1.left_stick_y;
        horizontal = gamepad1.left_stick_x;
        pivot = gamepad1.right_stick_x;

        if (gamepad1.left_bumper) {
            RFMotor.setPower(0.4 * (pivot + (vertical + horizontal)));
            RBMotor.setPower(0.4 * (pivot + (vertical - horizontal)));
            LFMotor.setPower(0.4 * (-pivot + (vertical - horizontal)));
            LBMotor.setPower(0.4 * (-pivot + (vertical + horizontal)));
        } else {
            RFMotor.setPower(pivot + (vertical + horizontal));
            RBMotor.setPower(pivot + (vertical - horizontal));
            LFMotor.setPower(-pivot + (vertical - horizontal));
            LBMotor.setPower(-pivot + (vertical + horizontal));
        }
    }

    // Intake method
    public void Intake(){
        if ((gamepad2.left_trigger) > 0.5 && t1.seconds() > 0.5) {
            Intake.setPower(1);
        } else if((gamepad2.right_stick_button)) {
            Intake.setPower(-1);
        } else if (gamepad2.left_stick_button) {
            Intake.setPower(0);
        }

    }

    // Arm method
    public void Arm() {
        if ((gamepad2.b)) {
            Arm.setPosition(0.1);
        }
        if ((gamepad2.a)) {
            Arm.setPosition(0.9);
        }
        if ((gamepad2.x)) {
            Arm.setPosition(0.7);
        }
        if ((gamepad2.y)) {
            Arm.setPosition(0.35);
        }

        telemetry.addData("Arm Position", Arm.getPosition());
        telemetry.update();

    }


    //sleep methods
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


    @Override
    public void init() {
        RFMotor = hardwareMap.get(DcMotor.class, "RFMotor");
        RBMotor = hardwareMap.get(DcMotor.class, "RBMotor");
        LFMotor = hardwareMap.get(DcMotor.class, "LFMotor");
        LBMotor = hardwareMap.get(DcMotor.class, "LBMotor");
        Intake = hardwareMap.get(DcMotor.class, "Intake");
        Arm = hardwareMap.get(Servo.class, "Arm");

        LFMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        LBMotor.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    @Override
    public void init_loop() {

        Arm.setPosition(0.1);
        telemetry.addData("Arm Position" , Arm.getPosition());
        telemetry.update();


    }

    @Override
    public void start(){
        t2.reset();

    }

    @Override
    public void loop() {
        moveDriveTrain();
        Intake();
        Arm();
        telemetry.update();

    }
}