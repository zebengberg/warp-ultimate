package org.firstinspires.ftc.teamcode.fourwheel;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class FourWheelAuto extends FourWheelTele {

    @Override
    public void runOpMode() {
        goForward(1000);
        goForward(2000);
    }

    private void goForward(int position) {
        for (DcMotor motor : motors) {
            motor.setTargetPosition(position);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }


}
