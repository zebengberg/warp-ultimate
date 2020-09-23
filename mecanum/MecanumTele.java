package org.firstinspires.ftc.teamcode.mecanum;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class MecanumTele extends LinearOpMode {
    private DcMotor frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;
    private DcMotor[] motors = {frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor};

    @Override
    public void runOpMode() {
        frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        backRightMotor = hardwareMap.dcMotor.get("backRightMotor");

//        for (DcMotor motor : motors) {
//            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//            motor.setDirection(DcMotor.Direction.FORWARD);
//        }

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
            if (max > 0.1) {
                frontLeftPower /= max;
                backLeftPower /= max;
                frontRightPower /= max;
                backRightPower /= max;
            }

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);

            telemetry.addData("x", x);
            telemetry.addData("y", y);
            telemetry.addData("r", r);
            telemetry.update();

        }
    }
}