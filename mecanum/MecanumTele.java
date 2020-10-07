package org.firstinspires.ftc.teamcode.mecanum;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
public class MecanumTele extends LinearOpMode {

    // public allows access from derived class
    public DcMotorEx frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor;
    public DcMotorEx[] motors;

    public MecanumTele() {
        frontLeftMotor = (DcMotorEx)hardwareMap.dcMotor.get("frontLeftMotor");
        frontRightMotor = (DcMotorEx)hardwareMap.dcMotor.get("frontRightMotor");
        backLeftMotor = (DcMotorEx)hardwareMap.dcMotor.get("backLeftMotor");
        backRightMotor = (DcMotorEx)hardwareMap.dcMotor.get("backRightMotor");
        motors = new DcMotorEx[] {frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor};

        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightMotor.setDirection(DcMotor.Direction.FORWARD);

        for (DcMotor motor : motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }



    @Override
    public void runOpMode() {
        double maxSpeed = 1.0;
        boolean isPressingX = false;


        waitForStart();
        while (opModeIsActive()) {
            double x = gamepad1.left_stick_x;
            double y = -gamepad1.left_stick_y;
            double r = gamepad1.right_stick_x;


            double frontLeftPower = y + x + r;
            double backLeftPower = y - x + r;
            double frontRightPower = y - x - r;
            double backRightPower = y + x - r;

            double max = Math.max(Math.abs(frontLeftPower), Math.abs(backLeftPower));
            max = Math.max(Math.abs(frontRightPower), max);
            max = Math.max(Math.abs(backRightPower), max);
            if (max > 1) {
                frontLeftPower /= max;
                backLeftPower /= max;
                frontRightPower /= max;
                backRightPower /= max;
            }

            // slow mode; gamepad1.x was just released
            if (!gamepad1.x && isPressingX) {
                if (maxSpeed == 1.0) {
                    maxSpeed = 0.5;
                } else {
                    maxSpeed = 1.0;
                }
            }
            isPressingX = gamepad1.x;

            frontLeftPower *= maxSpeed;
            backLeftPower *= maxSpeed;
            frontRightPower *= maxSpeed;
            backRightPower *= maxSpeed;



            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);

            telemetry.addData("x", x);
            telemetry.addData("y", y);
            telemetry.addData("r", r);
            telemetry.addData("max", max);
            telemetry.addData("isPressingX", isPressingX);
            telemetry.addData("frontLeftPower", frontLeftPower);
            telemetry.addData("backLeftPower", backLeftPower);
            telemetry.addData("frontRightPower", frontRightPower);
            telemetry.addData("backRightPower", backRightPower);
            telemetry.update();

        }
    }
}