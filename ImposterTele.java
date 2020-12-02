package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Imposter", group="Imposter")
public class ImposterTele extends LinearOpMode {
    MecanumRobot robot;// = new MecanumRobot(hardwareMap, telemetry, "imposter");


    @Override
    public void runOpMode() {

        robot = new ImposterRobot(hardwareMap, telemetry);
        boolean xToggle = true;
        double maxSpeed = 0.5;


        waitForStart();
        while (opModeIsActive()) {
            double x = gamepad1.left_stick_x;
            double y = -gamepad1.left_stick_y;
            double r = gamepad1.right_stick_x;
            robot.move(x, y, r, maxSpeed);


            if (gamepad1.x) {
                if (xToggle) {
                    if (maxSpeed == 0.5) {
                        maxSpeed = 0.8;
                    } else {
                        maxSpeed = 0.5;
                    }
                    xToggle = false;
                }
            } else {
                xToggle = true;
            }


            if (gamepad1.y) {
                robot.intake(0.8);
            } else if (gamepad1.b) {
                robot.intake(-0.8);
            } else {
                robot.intake(0.0);
            }

            if (gamepad1.left_bumper) {
                robot.shoot(0.5);
            } else if (gamepad1.right_bumper) {
                robot.shoot(0.7);
            } else {
                robot.shoot(0.0);
            }

            robot.moveArm(gamepad1.right_trigger);
            if (gamepad1.right_trigger == 0.0) {
                robot.moveArm(-gamepad1.left_trigger);
            }

            robot.printStatus();
        }
    }
}