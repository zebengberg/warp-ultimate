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


@Autonomous
public class MecanumAutoBase extends LinearOpMode {

    public BNO055IMU imu;
    public PIDFCoefficients pidf1, pidf2;

    public MecanumAutoBase() {
        super.init();

        // imu initialization
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.loggingEnabled = false;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        telemetry.addData("Status", "Calibrating gyro...");
        telemetry.update();

        while (!opModeIsActive() && !imu.isGyroCalibrated()) { idle(); }
        telemetry.addData("Status", imu.getCalibrationStatus().toString());
        telemetry.addData("Status", "Initialized!");
        telemetry.update();

        // PIDF coefficients
        pidf1 = frontLeftMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        pidf2 = new PIDFCoefficients(1.0, 0.2, 0.2, 1.0);
    }


    @Override
    public void runOpMode() {
        waitForStart();
        goForward(2000);
        goBackward(2000);
        spin(90);
        goForward(1000);
        goBackward(1000);

    }

    public void goForward(int position) {
        for (DcMotor motor : motors) {
            motor.setTargetPosition(position);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setPower(0.25);
        }

        for (DcMotorEx motor : motors) {
            motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf1);
        }

        while (frontLeftMotor.isBusy() && frontRightMotor.isBusy() && backLeftMotor.isBusy() && backRightMotor.isBusy()) {
            printStatus();
        }
    }

    public void goBackward(int position) {
        for (DcMotor motor : motors) {
            motor.setTargetPosition(-position);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setPower(0.25);
        }

        for (DcMotorEx motor : motors) {
            motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf2);
        }

        while (frontLeftMotor.isBusy() && frontRightMotor.isBusy() && backLeftMotor.isBusy() && backRightMotor.isBusy()) {
            printStatus();
        }
    }

    public void printStatus() {
        for (DcMotor motor : motors) {
            telemetry.addData(motor.getDeviceName(), motor.getCurrentPosition());
            telemetry.addData(motor.getDeviceName(), motor.getTargetPosition());
        }
        telemetry.update();
    }

    public void spin(int targetAngle) {
        double error;
        do {

            Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            double currentAngle = angles.firstAngle;
            error = currentAngle - targetAngle;

            frontLeftMotor.setPower(-error);
            backLeftMotor.setPower(-error);
            frontRightMotor.setPower(error);
            backRightMotor.setPower(error);



            telemetry.addData("angle", currentAngle);
            telemetry.update();
        } while (Math.abs(error) > 30);
    }
}
