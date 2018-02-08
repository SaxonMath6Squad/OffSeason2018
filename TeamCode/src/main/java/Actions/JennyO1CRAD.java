package Actions;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import Actions.HardwareWrappers.ServoHandler;
import Actions.HardwareWrappers.SpoolMotor;
import MotorControllers.MotorController;


/**
 * Created by Jeremy on 8/23/2017.
 */

/*
    A class to handle our RAD for the relic - Relic Arm Device
 */
public class JennyO1CRAD {
    ServoHandler frontRelicGrabber;
    SpoolMotor RADExtender;
    HardwareMap hardwareMap;
    final double RELEASE_FRONT = 179;
    final double GRAB_FRONT = 111;
    final double RELEASE_BACK = 40;
    final double GRAB_BACK = 100;
    final double STOP_RAD = 87;
    final double RELEASE_RAD = 8;
    final double ZERO_ROTATION = 0;
    final double FLIP_ROTATION = 170;
    final double PERPENDICULAR_ROTATION = 80;

    public JennyO1CRAD(HardwareMap hw) throws InterruptedException {
        hardwareMap = hw;
        frontRelicGrabber = new ServoHandler("frontRelicGrabber", hardwareMap);


        try {
            RADExtender = new SpoolMotor(new MotorController("RADExtender", "ActionConfig/RelicExtender.json", hardwareMap),
                    35, 5, 100, hardwareMap);
        } catch (Exception e) {
            throw new RuntimeException(e);
        }
        RADExtender.setDirection(DcMotorSimple.Direction.FORWARD);
        RADExtender.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRelicGrabber.setServoRanges(GRAB_FRONT-1, RELEASE_FRONT+1);
        frontRelicGrabber.setDegree(GRAB_FRONT);

    }

    public int extendRAD(){
        RADExtender.extendWithPower();
        return 0;
    }

    public int retractRAD(){
        RADExtender.retractWithPower();
        return 0;
    }

    public int pauseRADExtender(){
        RADExtender.setPower(0);
        return 0;
    }

    public int grabFrontRelic(){
        frontRelicGrabber.setDegree(GRAB_FRONT);
        return 0;
    }

    public int releaseFrontRelic(){
        frontRelicGrabber.setDegree(RELEASE_FRONT);
        return 0;
    }

    public int setFrontGrabberPosition(double positionInDeg){
        frontRelicGrabber.setDegree(positionInDeg);
        return 0;
    }

    public int kill(){
        RADExtender.kill();
        frontRelicGrabber.setPosition(frontRelicGrabber.getPosition());
        return 0;
    }
}
