package org.firstinspires.ftc.teamcode.mecanum;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class MecanumRobot {

    public DcMotorEx frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor;
    public DcMotorEx[] motors;

    HardwareMap hwMap;
    Telemetry tel;


    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;

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
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void printStatus(Telemetry atel) {
        tel = atel;
        tel.addData("hello", "world");
        tel.update();
    }
}
