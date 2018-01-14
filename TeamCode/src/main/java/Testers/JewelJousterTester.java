package Testers;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import Actions.JewelJouster;
import Autonomous.REVColorDistanceSensorController;

/**
 * Created by robotics on 1/10/18.
 */
@Autonomous(name = "JewelJouster Color Sensor Test", group = "Sensor")

public class JewelJousterTester extends LinearOpMode{
    @Override
    public void runOpMode() throws InterruptedException {
        JewelJouster jouster = new JewelJouster("jewelJouster",hardwareMap);
        waitForStart();
        jouster.setPosition(JewelJouster.EXTENDION_MODE.HIT);
        while(opModeIsActive()){
            REVColorDistanceSensorController.color jewelColor = jouster.getJewelColor();
            telemetry.addData("Dist",""+jouster.getDistance());
            switch (jewelColor){
                case RED:
                    telemetry.addData("JewelColor","RED");
                    break;
                case BLUE:
                    telemetry.addData("JewelColor","BLUE");
                    break;
                case UNKNOWN:
                    telemetry.addData("JewelColor","UNKNOWN");
                    break;
                case NOT_IN_RANGE:
                    telemetry.addData("JewelColor","NOT_IN_RANGE");
                    break;
            }
            telemetry.update();
        }
    }
}
