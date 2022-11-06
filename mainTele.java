package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

//@Disabled
@TeleOp (name="Main")
public class mainTele extends OpMode {

    DcMotor A;
    DcMotor B;
    DcMotor C;
    DcMotor D;
    DcMotor ele;
    DcMotor arm;
    Servo claw;
    // * add variable for elevator motor (use DcMotorEx for encoders)

        public void init() {
            A = hardwareMap.get(DcMotor.class, "motorA");
            B = hardwareMap.get(DcMotor.class, "motorB");
            C = hardwareMap.get(DcMotor.class, "motorC");
            D = hardwareMap.get(DcMotor.class, "motorD");
            ele = hardwareMap.get(DcMotor.class, "ele");
            arm = hardwareMap.get(DcMotor.class, "arm");
            claw = hardwareMap.get(Servo.class, "clawServo");

            B.setDirection(DcMotor.Direction.REVERSE);
            C.setDirection(DcMotor.Direction.REVERSE);
        }


        public void loop() {

            double y = -gamepad1.left_stick_y;
            double x = -gamepad1.left_stick_x;
            double h = -gamepad1.right_stick_x * 0.6;
            double newX = (x * 0.707) - (y * 0.707);
            double newY = (x * 0.707) + (y * 0.707);
            double c = 0.5;

            double aSpeed = (newX + h) * c;
            double bSpeed = (-newY - h) * c;
            double cSpeed = (newX - h) * c;
            double dSpeed = (-newY + h) * c;

            A.setPower(aSpeed);
            B.setPower(bSpeed);
            C.setPower(cSpeed);
            D.setPower(dSpeed);

            boolean aBtn = gamepad1.a;
            boolean leftBumper = gamepad1.left_bumper;
            boolean rightBumper = gamepad1.right_bumper;
            boolean yBtn = gamepad1.y;
            boolean bBtn = gamepad1.b;
            boolean dpadUp = gamepad1.dpad_up;
            boolean dpadRight = gamepad1.dpad_right;
            boolean dpadLeft = gamepad1.dpad_left;
            boolean dpadDown = gamepad1.dpad_down;

            if (rightBumper) {
                ele.setPower(1);
            } else {
                ele.setPower(0);
            }

            if (leftBumper) {
                ele.setPower(-1);
            } else {
                ele.setPower(0);
            }

            if (yBtn) {
                arm.setPower(1);
            } else {
                arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                arm.setPower(0);
            }

            if (bBtn) {
                arm.setPower(-1);
            } else {
                arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                arm.setPower(0);
            }

            if (aBtn) {
                claw.setPosition(0.2);
            } else {
                claw.setPosition(0);
            }
        }
    }
