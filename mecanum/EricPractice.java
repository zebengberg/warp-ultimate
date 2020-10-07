package org.firstinspires.ftc.teamcode.mecanum;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous
public class EricPractice extends LinearOpMode {
    private DcMotor frontLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backLeftMotor;
    private DcMotor backRightMotor;
    private DcMotor[] motors;
    private BNO055IMU imu;


    @Override
    public void runOpMode() {
        // DC motors for holonomic drive.
        frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        backRightMotor = hardwareMap.dcMotor.get("backRightMotor");
        motors = new DcMotor[] {frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor};


        for (DcMotor motor : motors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
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


        telemetry.speak("roger roger ROGER");

        telemetry.speak("bri bri brian");


        waitForStart();
        goForward();
//        if (opModeIsActive()) {
//            goForward();
//        }



        //spin();
        //goBackward();




    }

    public void goForward() {

        telemetry.speak("go forward");
        telemetry.addData("moving", "forward");
        telemetry.update();



        for (DcMotor motor : motors) {
            motor.setTargetPosition(10000);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        while (frontLeftMotor.isBusy() && frontRightMotor.isBusy() && backLeftMotor.isBusy() && backRightMotor.isBusy()) {
            telemetry.speak("going forward");

        }


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
