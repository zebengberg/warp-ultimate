package org.firstinspires.ftc.teamcode.mecanum;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name="MecanumTele", group="Mecanum")
public class MecanumTele extends LinearOpMode {
    MecanumRobot robot = new MecanumRobot();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap, telemetry);

        double maxSpeed = 1.0;
        boolean isPressingX = false;


        waitForStart();
        while (opModeIsActive()) {
            double x = gamepad1.left_stick_x;
            double y = -gamepad1.left_stick_y;
            double r = gamepad1.right_stick_x;


            double frontLeftPower = y - x + r;
            double backLeftPower = y + x + r;
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



            robot.frontLeftMotor.setPower(frontLeftPower);
            robot.backLeftMotor.setPower(backLeftPower);
            robot.frontRightMotor.setPower(frontRightPower);
            robot.backRightMotor.setPower(backRightPower);


            robot.intake(gamepad1.right_trigger);



            robot.printStatus(telemetry);


        }

    }
}