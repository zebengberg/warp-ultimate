package org.firstinspires.ftc.teamcode.fourwheel;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name="FourWheelTele", group="FourWheel")
public class FourWheelTele extends LinearOpMode {
    FourWheelRobot robot = new FourWheelRobot();

    private boolean isRightBumperPressed = false;
    private boolean isLeftBumperPressed = false;
    private double shooterSpeed = 0.0;


    @Override
    public void runOpMode() {
        robot.init(hardwareMap, telemetry);


        double maxSpeed = 0.6;

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

            // shooting and loading
            shoot();
            robot.load(-gamepad1.left_trigger);

            maxSpeed = 0.6;
            frontLeftPower *= maxSpeed;
            backLeftPower *= maxSpeed;
            frontRightPower *= maxSpeed;
            backRightPower *= maxSpeed;

            robot.frontLeftMotor.setPower(frontLeftPower);
            robot.backLeftMotor.setPower(backLeftPower);
            robot.frontRightMotor.setPower(frontRightPower);
            robot.backRightMotor.setPower(backRightPower);

            robot.printStatus(telemetry);
        }
    }

    private void shoot() {
        double FAST_SPEED = 0.56;
        double SLOW_SPEED = 0.50;

        if (gamepad1.left_bumper && !isLeftBumperPressed) {
            if (shooterSpeed == SLOW_SPEED) {
                shooterSpeed = 0.0;
            } else {
                shooterSpeed = SLOW_SPEED;
            }
        }
        else if (gamepad1.right_bumper && !isRightBumperPressed) {
            if (shooterSpeed == FAST_SPEED) {
                shooterSpeed = 0.0;
            } else {
                shooterSpeed = FAST_SPEED;
            }
        }
        isLeftBumperPressed = gamepad1.left_bumper;
        isRightBumperPressed = gamepad1.right_bumper;
        robot.shoot(shooterSpeed);
    }

}