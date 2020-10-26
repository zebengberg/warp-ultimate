package org.firstinspires.ftc.teamcode.mecanum;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@Autonomous(name="MecanumAutoBase", group="Mecanum")
public class MecanumAutoBase extends LinearOpMode {

    public enum Direction { FORWARD, BACKWARD, LEFT, RIGHT }
    public enum RingSize { ZERO, ONE, FOUR }
    public enum StartingOrientation { BLUELEFT, BLUERIGHT, REDLEFT, REDRIGHT }
    public enum Goal { WOBBLE, PICKUPRINGS, SHOOTTOWER, SHOOTPEGS, PARK }




    MecanumRobot robot = new MecanumRobot();
    public Goal goal;
    public StartingOrientation orientation;
    public PIDFCoefficients pidf1, pidf2;

//    MecanumAutoBase(StartingOrientation orientation, Goal goal) {
//        this.orientation = orientation;
//        this.goal = goal;
//    }



    @Override
    public void runOpMode() {
        robot.init(hardwareMap, telemetry);

        // PIDF coefficients
        pidf1 = robot.frontLeftMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        pidf2 = new PIDFCoefficients(1.0, 0.2, 0.2, 1.0);

        // put in parameters for testing here
        goal = Goal.SHOOTPEGS;
        orientation = StartingOrientation.BLUERIGHT;


        waitForStart();

        if (goal == Goal.PARK) {
            sleep(2500);
            move(Direction.FORWARD, 183, pidf1);
        } else if (orientation == StartingOrientation.BLUELEFT && goal == Goal.WOBBLE) {
            move(Direction.FORWARD, 98, pidf1);
            RingSize rings = countRings();
            telemetry.speak("I see " + rings.toString());

            switch (rings) {
                case FOUR:
                    move(Direction.FORWARD, 182, pidf1);
                    move(Direction.LEFT, 25, pidf1);
                    break;

                case ONE:
                    move(Direction.FORWARD, 122, pidf1);
                    move(Direction.RIGHT, 25, pidf1);
                    break;

                case ZERO:
                    move(Direction.FORWARD, 62, pidf1);
                    move(Direction.LEFT, 25, pidf1);
                    break;
            }
        } else if (orientation == StartingOrientation.BLUERIGHT && goal == Goal.SHOOTPEGS) {
            move(Direction.FORWARD, 155, pidf1);
            move(Direction.RIGHT, 33, pidf1);
        }
    }

    public RingSize countRings() {
        if (orientation == StartingOrientation.BLUELEFT || orientation == StartingOrientation.REDLEFT) {
            double dist = robot.rightDistance.getDistance(DistanceUnit.CM);
            if (dist < 15) {
                return RingSize.FOUR;
            } else if (dist < 22) {
                return RingSize.ONE;
            } else {
                return RingSize.ZERO;
            }
        }

        return RingSize.ZERO;
    }

    public int cmToEncoderUnits(double cm) {
        return (int) (5000 * cm / 205);
    }

    public void move(Direction direction, double cm, PIDFCoefficients pidf) {

        int encoderUnits = cmToEncoderUnits(cm);
        for (DcMotor motor : robot.motors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        double x = 0;
        double y = 0;
        if (direction == Direction.FORWARD) {
            y = 1;
        } else if (direction == Direction.RIGHT) {
            x = -1;
        } else if (direction == Direction.LEFT) {
            x = 1;
        } else if (direction == Direction.BACKWARD) {
            y = -1;
        }

        double frontLeftPower = y - x;
        double frontRightPower = y + x;
        double backLeftPower = y + x;
        double backRightPower = y - x;

        robot.frontLeftMotor.setTargetPosition((int)Math.signum(frontLeftPower) * encoderUnits);
        robot.frontRightMotor.setTargetPosition((int)Math.signum(frontRightPower) * encoderUnits);
        robot.backLeftMotor.setTargetPosition((int)Math.signum(backLeftPower) * encoderUnits);
        robot.backRightMotor.setTargetPosition((int)Math.signum(backRightPower) * encoderUnits);

        double power = 0.8;
        for (DcMotor motor : robot.motors) {
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setPower(power);
        }



        while (robot.frontLeftMotor.isBusy() && robot.frontRightMotor.isBusy() &&
                robot.backLeftMotor.isBusy() && robot.backRightMotor.isBusy()) {
            robot.printStatus(telemetry);
            idle();
        }
    }
}