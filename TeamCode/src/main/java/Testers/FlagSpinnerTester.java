package Testers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import Actions.JennyFlagController;

/**
 * Created by robotics on 1/6/18.
 */
@TeleOp(name="Flag Tester", group="Linear Opmode")  // @Autonomous(...) is the other common choice
public class FlagSpinnerTester extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        JennyFlagController flag = new JennyFlagController(hardwareMap);
        waitForStart();
        flag.startFlag();
        while (opModeIsActive());
        flag.stopFlag();

    }
}
