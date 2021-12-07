package org.firstinspires.ftc.teamcode.teleOp;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
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
    private DcMotor Shooter;
    private DcMotor intake;

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
        if(gamepad1.left_bumper){
            LFMotor.setPower(0.6*(gamepad1.right_stick_y));
            LBMotor.setPower(0.6*(gamepad1.right_stick_y));
            RFMotor.setPower(0.6*(-gamepad1.left_stick_y));
            RBMotor.setPower(0.6*(-gamepad1.left_stick_y));

            telemetry.addData("joystick", gamepad1.right_stick_y);
            telemetry.addData("joystick", gamepad1.left_stick_y);

        }else {
            LFMotor.setPower(gamepad1.right_stick_y);
            LBMotor.setPower(gamepad1.right_stick_y);
            RFMotor.setPower(-gamepad1.left_stick_y);
            RBMotor.setPower(-gamepad1.left_stick_y);

            telemetry.addData("joystick", gamepad1.right_stick_y);
            telemetry.addData("joystick", gamepad1.left_stick_y);
        }
    }

    //Push 3 times method

    //    Intake method
    public void Intake(){
        if ((gamepad1.left_trigger) > 0.5 && t1.seconds() > 0.5) {
            intake.setPower(1);
        } else if((gamepad1.right_stick_button)) {
            intake.setPower(-1);
        } else if (gamepad1.b) {
            intake.setPower(0);
        }

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

    }

    @Override
    public void init_loop() {





    }

    @Override
    public void start(){
        t2.reset();

    }

    @Override
    public void loop() {
        moveDriveTrain();


    }

}