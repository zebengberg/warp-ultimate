package org.firstinspires.ftc.teamcode.mecanum;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class MecanumRobot {

    public DcMotorEx frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor;
    public DcMotorEx[] motors;
    public BNO055IMU imu;
    public ColorSensor leftColor, rightColor;

    HardwareMap hwMap;
    Telemetry tel;


    public void init(HardwareMap ahwMap, Telemetry atel) {
        hwMap = ahwMap;
        tel = atel;

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

        // imu initialization
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.loggingEnabled = false;
        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        // color sensor initialization
        leftColor = hwMap.get(ColorSensor.class, "leftColor");
        rightColor = hwMap.get(ColorSensor.class, "rightColor");

        // saying its name
        tel.speak("CREW MATE IS NOW SELF_AWARE");
    }

    public void printStatus(Telemetry atel) {
        tel = atel;
        tel.addData("frontLeftMotor", frontLeftMotor.getPower());
        tel.addData("frontRightMotor", frontRightMotor.getPower());
        tel.addData("backLeftMotor", backLeftMotor.getPower());
        tel.addData("backRightMotor", backRightMotor.getPower());
        tel.addData("leftColor", leftColor.red() + " " + leftColor.green() +
                " " + leftColor.blue());
        tel.addData("rightColor", rightColor.red() + " " + rightColor.green() +
                " " + rightColor.blue());
        tel.update();
    }
}
