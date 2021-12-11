package org.firstinspires.ftc.teamcode.FreightFrenzyLeage0;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp
@Disabled
public class FreightFrenzyTeleOp extends OpMode {
    private DcMotor RFMotor;
    private DcMotor LFMotor;
    private DcMotor RBMotor;
    private DcMotor LBMotor;

    ElapsedTime t1 = new ElapsedTime();

    public void moveDriveTrain() {
        double vertical = 0;
        double horizontal = 0;
        double pivot = 0;
        vertical = gamepad1.left_stick_y;
        horizontal = gamepad1.left_stick_x;
        pivot = gamepad1.right_stick_x;



        if (gamepad1.left_bumper) {
            RFMotor.setPower(0.25 * (pivot + (vertical + horizontal)));
            RBMotor.setPower(0.25 * (pivot + (vertical - horizontal)));
            LFMotor.setPower(0.25 * (-pivot + (vertical - horizontal)));
            LBMotor.setPower(0.25 * (-pivot + (vertical + horizontal)));
        } else {
            RFMotor.setPower(pivot + (vertical + horizontal));
            RBMotor.setPower(pivot + (vertical - horizontal));
            LFMotor.setPower(-pivot + (vertical - horizontal));
            LBMotor.setPower(-pivot + (vertical + horizontal));
        }
    }



    @Override
    public void init() {
        RFMotor = hardwareMap.get(DcMotor.class, "RFMotor");
        LFMotor = hardwareMap.get(DcMotor.class, "LFMotor");
        LBMotor = hardwareMap.get(DcMotor.class, "LBMotor");
        RBMotor = hardwareMap.get(DcMotor.class, "RBMotor");

        RFMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        RBMotor.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    @Override
    public void init_loop() {

    }

    @Override
    public void loop() {
        moveDriveTrain();

        telemetry.addData("Left Joystick: ", gamepad1.left_stick_y);
        telemetry.addData("Right Joystick: ", gamepad1.right_stick_y);
        telemetry.update();
    }
}
