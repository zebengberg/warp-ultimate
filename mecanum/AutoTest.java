package org.firstinspires.ftc.teamcode.mecanum;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


@Autonomous
public class AutoTest extends LinearOpMode {
    private DcMotorEx frontLeftMotor;
    private DcMotorEx frontRightMotor;
    private DcMotorEx backLeftMotor;
    private DcMotorEx backRightMotor;
    private DcMotorEx[] motors;
    private BNO055IMU imu;



    @Override
    public void runOpMode() {
        // DC motors for holonomic drive.
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
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        PIDFCoefficients pidOld = frontLeftMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        PIDFCoefficients pidfNew = new PIDFCoefficients(1.0, 0.2, 0.2, 1.0);

        for (DcMotorEx motor : motors) {
            motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfNew);
        }
        

        // IMU DEVICE
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.loggingEnabled = false;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);


        // Make sure the imu gyro is calibrated before continuing.
        telemetry.addData("Status", "Calibrating gyro...");
        telemetry.update();

        while (!opModeIsActive() && !imu.isGyroCalibrated()) { idle(); }
        telemetry.addData("Status", imu.getCalibrationStatus().toString());
        telemetry.addData("Status", "Initialized!");
        telemetry.update();


        telemetry.speak("brian");


        waitForStart();
        goForward();

    }

    public void goForward() {

        for (DcMotor motor : motors) {
            motor.setTargetPosition(5000);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setPower(0.25);
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

    public void goBackward() {
        for (DcMotor motor : motors) {
            motor.setTargetPosition(-10000);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        while (frontLeftMotor.isBusy() && frontRightMotor.isBusy() && backLeftMotor.isBusy() && backRightMotor.isBusy()) {
            telemetry.speak("going back");
            telemetry.update();
        }
    }

    public void spin() {
        while (true) {
            for (DcMotor motor : motors) {
                motor.setPower(1);
            }

            Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
            double currentAngle = angles.firstAngle;

            telemetry.addData("angle", currentAngle);
            telemetry.update();

            if (currentAngle < 2 * Math.PI) {
                break;
            }
        }
    }
}
