package org.firstinspires.ftc.teamcode.fourwheel;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="FourWheelAuto", group="FourWheel")
public class FourWheelAuto extends LinearOpMode {
    FourWheelRobot robot = new FourWheelRobot();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap, telemetry);

        waitForStart();
        robot.goForward(1000);
        robot.goForward(2000);
    }
}
