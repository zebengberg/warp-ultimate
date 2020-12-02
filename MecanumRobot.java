package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;


public class MecanumRobot {

    public DcMotorEx frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor;
    public DcMotorEx[] motors;
    public DcMotor intakeMotor;
    public DcMotor leftShooter, rightShooter;
    public BNO055IMU imu;
    CRServo arm;

    HardwareMap hwMap;
    Telemetry telemetry;
    String name;


    public MecanumRobot(HardwareMap hwMap, Telemetry telemetry, String name) {
        this.hwMap = hwMap;
        this.telemetry = telemetry;
        this.name = name;


        // motor initialization
        frontLeftMotor = (DcMotorEx)hwMap.dcMotor.get("frontLeftMotor");
        frontRightMotor = (DcMotorEx)hwMap.dcMotor.get("frontRightMotor");
        backLeftMotor = (DcMotorEx)hwMap.dcMotor.get("backLeftMotor");
        backRightMotor = (DcMotorEx)hwMap.dcMotor.get("backRightMotor");
        motors = new DcMotorEx[] {frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor};

        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightMotor.setDirection(DcMotor.Direction.FORWARD);

        for (DcMotor motor : motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        // input motor initialization
        intakeMotor = hwMap.dcMotor.get("intakeMotor");
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setDirection(DcMotor.Direction.REVERSE);

        // shooter motor initialization
        leftShooter = hwMap.dcMotor.get("leftShooter");
        rightShooter = hwMap.dcMotor.get("rightShooter");
        leftShooter.setDirection(DcMotor.Direction.FORWARD);
        rightShooter.setDirection(DcMotor.Direction.REVERSE);

        // imu initialization
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.loggingEnabled = false;
        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        // arm initialization
        hwMap.servo.get("arm");


        telemetry.speak(name + " IS LOOKING HELLA SUSS");
    }

    public void intake(double power) {
        intakeMotor.setPower(power);
    }

    public void shoot(double power) {
        leftShooter.setPower(power);
        rightShooter.setPower(power);
    }

    public void move(double x, double y, double r, double maxSpeed) {
        double frontLeftPower = y + x + r;
        double backLeftPower = y - x + r;
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

        frontLeftPower *= maxSpeed;
        backLeftPower *= maxSpeed;
        frontRightPower *= maxSpeed;
        backRightPower *= maxSpeed;

        frontLeftMotor.setPower(frontLeftPower);
        backLeftMotor.setPower(backLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backRightMotor.setPower(backRightPower);
    }

    public void moveArm(double power) {
        arm.setPower(power);
    }

    public void printStatus() {
        telemetry.addData("frontLeftMotor", frontLeftMotor.getPower());
        telemetry.addData("frontRightMotor", frontRightMotor.getPower());
        telemetry.addData("backLeftMotor", backLeftMotor.getPower());
        telemetry.addData("backRightMotor", backRightMotor.getPower());
        telemetry.addData("intakeMotor", intakeMotor.getPower());
        telemetry.addData( "imu", imu.getAngularOrientation().firstAngle);
        telemetry.update();
    }

}
