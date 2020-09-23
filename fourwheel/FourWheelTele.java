package org.firstinspires.ftc.teamcode.fourwheel;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class FourWheelTele extends LinearOpMode {
    private DcMotor frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;
    private DcMotor[] motors = {frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor};

    @Override
    public void runOpMode() {
        frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        backRightMotor = hardwareMap.dcMotor.get("backRightMotor");

        waitForStart();
        while (opModeIsActive()) {
            double x = gamepad1.left_stick_x * 1.5;
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

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);
        }
    }
}