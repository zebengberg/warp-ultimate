package org.firstinspires.ftc.teamcode.fourwheel;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class FourWheelRobot {

    public DcMotor frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor;
    public DcMotor[] driveMotors;

    public DcMotor leftShooter, rightShooter, loadingMotor;
    public DcMotor[] shooterMotors;

    HardwareMap hwMap;
    Telemetry tel;


    public void init(HardwareMap ahwMap, Telemetry atel) {
        hwMap = ahwMap;
        tel = atel;

        // drive motor initialization
        frontLeftMotor = hwMap.dcMotor.get("frontLeftMotor");
        frontRightMotor = hwMap.dcMotor.get("frontRightMotor");
        backLeftMotor = hwMap.dcMotor.get("backLeftMotor");
        backRightMotor = hwMap.dcMotor.get("backRightMotor");
        driveMotors = new DcMotor[] {frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor};

        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightMotor.setDirection(DcMotor.Direction.FORWARD);

        for (DcMotor motor : driveMotors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        // shooter motor initialization
        leftShooter = hwMap.dcMotor.get("leftShooter");
        rightShooter = hwMap.dcMotor.get("rightShooter");
        loadingMotor = hwMap.dcMotor.get("loadingMotor");
        shooterMotors = new DcMotor[] {leftShooter, rightShooter};

        leftShooter.setDirection(DcMotor.Direction.FORWARD);
        rightShooter.setDirection(DcMotor.Direction.REVERSE);
        loadingMotor.setDirection(DcMotor.Direction.FORWARD);

        for (DcMotor motor : shooterMotors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        // loading motor initialization
        loadingMotor.setZeroPowerBehavior((DcMotor.ZeroPowerBehavior.BRAKE));


        // saying its name
        tel.speak("IMPOSTER IS NOW SELF_AWARE");
    }

    public void shoot(double speed) {
        for (DcMotor motor : shooterMotors) {
            motor.setPower(speed);
        }
    }

    public void load(double speed) {
        loadingMotor.setPower(speed);
    }

    public void goForward(int position) {
        for (DcMotor motor : driveMotors) {
            motor.setTargetPosition(position);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }

    public void printStatus(Telemetry atel) {
        tel = atel;
        tel.addData("frontLeftMotor", frontLeftMotor.getPower());
        tel.addData("frontRightMotor", frontRightMotor.getPower());
        tel.addData("backLeftMotor", backLeftMotor.getPower());
        tel.addData("backRightMotor", backRightMotor.getPower());
        tel.addData("leftShooter", leftShooter.getPower());
        tel.addData("rightShooter", rightShooter.getPower());
        tel.addData("loadingMotor", loadingMotor.getPower());
        tel.update();
    }
}
