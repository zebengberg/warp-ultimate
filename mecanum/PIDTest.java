//package org.firstinspires.ftc.teamcode.mecanum;
//
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.PIDCoefficients;
//
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
//import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
//import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
//
//@Autonomous
//public class PIDTest extends LinearOpMode {
//
//    private DcMotor frontLeftMotor;
//    private DcMotor backLeftMotor;
//    private DcMotor frontRightMotor;
//    private DcMotor backRightMotor;
//
//    int prevError = 0;
//    int totalError = 0;
//    double Kp = 0.2;
//    double Ki = 0.2;
//    double Kd = 0.2;
//
//    Orientation lastAngles = new Orientation();
//    double globalAngle, power = .30, correction, rotation;
//    boolean aButton, bButton, touched;
//    PIDCoefficients pidRotate, pidDrive;
//
//    @Override
//    public void runOpMode() {
//        frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
//        backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
//        frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
//        backRightMotor = hardwareMap.dcMotor.get("backRightMotor");
//
//        DcMotor[] motors = {frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor};
//
//        for (DcMotor motor : motors) {
//            telemetry.addData(motor.getDeviceName(), motor.getMotorType());
//            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//            motor.setDirection(DcMotor.Direction.FORWARD);
//            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        }
//
//        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
//        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
//        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
//        backRightMotor.setDirection(DcMotor.Direction.FORWARD);
//
//        waitForStart();
//
//        pidRotate = new PIDCoefficients(.003, .00003, 0);
//        pidDrive = new PIDCoefficients(.05, .0, 0);
//
//        telemetry.addData("Mode", "calibrating...");
//        telemetry.update();
//
//        while (!isStopRequested() && !imu.isGyroCalibrated())
//        {
//            sleep(50);
//            idle();
//        }
//
//        /*
//        while (opModeIsActive()) {
//
//            goForward(5000);
//
//        }
//        */
//    }
//
//    private void goForward(int target) {
//        int error = get_current() - target;
//        int deltaError = error - prevError;
//        totalError += error;
//        frontRightMotor.setPower(Kp * error + Kd * deltaError + Ki * totalError);
//        printPositions();
//    }
//
//    private void printPositions() {
//        telemetry.addData("front left", frontLeftMotor.getCurrentPosition());
//        telemetry.addData("front right", frontRightMotor.getCurrentPosition());
//        telemetry.addData("back right", backRightMotor.getCurrentPosition());
//        telemetry.addData("back left", backLeftMotor.getCurrentPosition());
//        telemetry.update();
//    }
//
//    private int get_current() {
//        return frontRightMotor.getCurrentPosition();
//    }
//
//
//
//}
