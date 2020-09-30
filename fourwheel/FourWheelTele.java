package org.firstinspires.ftc.teamcode.fourwheel;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class FourWheelTele extends LinearOpMode {

    double maxSpeed = 0.9;

    @Override
    public void runOpMode() {
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRightMotor");

        DcMotor[] motors = {frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor};

        for (DcMotor motor : motors) {
            telemetry.addData(motor.getDeviceName(), motor.getMotorType());
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setDirection(DcMotor.Direction.FORWARD);
        }

        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightMotor.setDirection(DcMotor.Direction.FORWARD);

        waitForStart();
        while (opModeIsActive()) {
            double x = gamepad1.left_stick_x;
            double y = -gamepad1.left_stick_y;



            double frontLeftPower = y + x * 1.5;
            double backLeftPower = y + x;
            double frontRightPower = y - x * 1.5;
            double backRightPower = y - x;

            double max = Math.max(Math.abs(frontLeftPower), Math.abs(backLeftPower));
            max = Math.max(Math.abs(frontRightPower), max);
            max = Math.max(Math.abs(backRightPower), max);
            if (max > 1) {
                frontLeftPower /= max;
                backLeftPower /= max;
                frontRightPower /= max;
                backRightPower /= max;
            }

            frontLeftPower *= maxSpeed;
            backLeftPower *= maxSpeed;
            frontRightPower *= maxSpeed;
            backRightPower *= maxSpeed;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);
        }
    }
}