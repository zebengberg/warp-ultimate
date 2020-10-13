package org.firstinspires.ftc.teamcode.fourwheel;


public class FourWheelAuto extends FourWheelTele {
    FourWheelRobot robot = new FourWheelRobot();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap, telemetry);

        waitForStart();
        robot.goForward(1000);
        robot.goForward(2000);
    }
}
