package org.firstinspires.ftc.teamcode.mecanum;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;







@Autonomous(name="MecanumAutoBase", group="Mecanum")
public class MecanumAutoBase extends LinearOpMode {
    MecanumRobot robot = new MecanumRobot();

    public PIDFCoefficients pidf1, pidf2;
    public enum Direction { FORWARD, BACKWARD, LEFT, RIGHT }



    @Override
    public void runOpMode() {
        robot.init(hardwareMap, telemetry);

        // PIDF coefficients
        pidf1 = robot.frontLeftMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        pidf2 = new PIDFCoefficients(1.0, 0.2, 0.2, 1.0);


        waitForStart();
        move(Direction.FORWARD, 10000, pidf1);
        move(Direction.RIGHT, 10000, pidf1);
        move(Direction.BACKWARD, 10000, pidf1);
        move(Direction.LEFT, 10000, pidf1);


        //goBackward(1000);
        // spin(90);
        sleep(5000);
        idle();
        // goForward(2000, pidf2);
        //goBackward(1000);

    }

    public void move(Direction direction, int distance, PIDFCoefficients pidf) {
        double x = 0;
        double y = 0;

        for (DcMotor motor : robot.motors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setTargetPosition(distance);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
//        for (DcMotorEx motor : robot.motors) {
//            motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
//        }

        if (direction == Direction.FORWARD) {
            y = 1;
        } else if (direction == Direction.RIGHT) {
            x = 1;
        } else if (direction == Direction.LEFT) {
            x = -1;
        } else if (direction == Direction.BACKWARD) {
            y = -1;
        }

        double power = 0.25;
        x *= power;
        y *= power;

        robot.frontLeftMotor.setPower(y - x);
        robot.frontRightMotor.setPower(y + x);
        robot.backLeftMotor.setPower(y + x);
        robot.backRightMotor.setPower(y - x);



        while (robot.frontLeftMotor.isBusy() && robot.frontRightMotor.isBusy() &&
                robot.backLeftMotor.isBusy() && robot.backRightMotor.isBusy()) {
            robot.printStatus(telemetry);
            idle();
        }
    }
}