package org.firstinspires.ftc.teamcode.mecanum;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


@Autonomous(name="MecanumAutoBase", group="Mecanum")
public class MecanumAutoBase extends LinearOpMode {
    MecanumRobot robot = new MecanumRobot();

    public BNO055IMU imu;
    public PIDFCoefficients pidf1, pidf2;






    @Override
    public void runOpMode() {
        robot.init(hardwareMap, telemetry);

        // PIDF coefficients
        pidf1 = robot.frontLeftMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        pidf2 = new PIDFCoefficients(1.0, 0.2, 0.2, 1.0);


        waitForStart();
        goForward(2000);
        goBackward(2000);
        spin(90);
        goForward(1000);
        goBackward(1000);

    }

    public void goForward(int position) {
        for (DcMotor motor : robot.motors) {
            motor.setTargetPosition(position);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setPower(0.25);
        }

        for (DcMotorEx motor : robot.motors) {
            motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf1);
        }

        while (robot.frontLeftMotor.isBusy() && robot.frontRightMotor.isBusy() &&
                robot.backLeftMotor.isBusy() && robot.backRightMotor.isBusy()) {
            robot.printStatus(telemetry);
        }
    }

    public void goBackward(int position) {
        for (DcMotor motor : robot.motors) {
            motor.setTargetPosition(-position);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setPower(0.25);
        }

        for (DcMotorEx motor : robot.motors) {
            motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf2);
        }

        while (robot.frontLeftMotor.isBusy() && robot.frontRightMotor.isBusy() &&
                robot.backLeftMotor.isBusy() && robot.backRightMotor.isBusy()) {
            robot.printStatus(telemetry);
        }
    }



    public void spin(int targetAngle) {
        double error;
        do {

            Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            double currentAngle = angles.firstAngle;
            error = currentAngle - targetAngle;

            robot.frontLeftMotor.setPower(-error);
            robot.backLeftMotor.setPower(-error);
            robot.frontRightMotor.setPower(error);
            robot.backRightMotor.setPower(error);



            telemetry.addData("angle", currentAngle);
            telemetry.update();
        } while (Math.abs(error) > 30);
    }
}