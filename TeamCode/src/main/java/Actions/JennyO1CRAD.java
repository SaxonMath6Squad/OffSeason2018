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
    ServoHandler backRelicGrabber;
    ServoHandler relicRotator;
    ServoHandler RADStopper;
    SpoolMotor RADExtender;
    HardwareMap hardwareMap;
    final double RELEASE_FRONT = 180;
    final double GRAB_FRONT = 100;
    final double RELEASE_BACK = 0;
    final double GRAB_BACK = 101;
    final double STOP_RAD = 87;
    final double RELEASE_RAD = 8;
    final double ZERO_ROTATION = 0;
    final double FLIP_ROTATION = 170;
    final double PERPENDICULAR_ROTATION = 80;

    public JennyO1CRAD(HardwareMap hw) throws InterruptedException {
        hardwareMap = hw;
        frontRelicGrabber = new ServoHandler("frontRelicGrabber", hardwareMap);
        backRelicGrabber = new ServoHandler("backRelicGrabber", hardwareMap);
        relicRotator = new ServoHandler("relicRotator", hardwareMap);
        RADStopper = new ServoHandler("RADStopper", hardwareMap);
        try {
            RADExtender = new SpoolMotor(new MotorController("RADExtender", "ActionConfig/RelicExtender.json", hardwareMap),
                    35, 5, 100, hardwareMap);
        } catch (Exception e) {
            throw new RuntimeException(e);
        }
        RADExtender.setDirection(DcMotorSimple.Direction.REVERSE);
        RADExtender.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RADStopper.setServoRanges(RELEASE_RAD-1, STOP_RAD+1);
        RADStopper.setDegree(RELEASE_RAD);
        frontRelicGrabber.setServoRanges(GRAB_FRONT-1, RELEASE_FRONT+1);
        frontRelicGrabber.setDegree(GRAB_FRONT);
        backRelicGrabber.setServoRanges(RELEASE_BACK-1, GRAB_BACK+1);
        backRelicGrabber.setDegree(RELEASE_BACK);
        relicRotator.setServoRanges(ZERO_ROTATION-1, FLIP_ROTATION+1);
        relicRotator.setDegree(ZERO_ROTATION);
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

    public int grabBackRelic(){
        backRelicGrabber.setDegree(GRAB_BACK);
        return 0;
    }

    public int releaseBackRelic(){
        backRelicGrabber.setDegree(RELEASE_BACK);
        return 0;
    }

    public int setBackGrabberPosition(double positionInDeg){
        backRelicGrabber.setDegree(positionInDeg);
        return 0;
    }

    public int zeroRotation(){
        relicRotator.setDegree(ZERO_ROTATION);
        return 0;
    }

    public int flipRotation(){
        relicRotator.setDegree(FLIP_ROTATION);
        return 0;
    }

    public int perpendicularRotation(){
        relicRotator.setDegree(PERPENDICULAR_ROTATION);
        return 0;
    }

    public int setRotationDegree(double positionInDeg){
        relicRotator.setDegree(positionInDeg);
        return 0;
    }

    public int activateStopper(){
        RADStopper.setDegree(STOP_RAD);
        return 0;
    }

    public int deactivateStopper(){
        RADStopper.setDegree(RELEASE_RAD);
        return 0;
    }

    public int setStopperDegree(double positionInDeg){
        RADStopper.setDegree(positionInDeg);
        return 0;
    }

    public int kill(){
        RADExtender.kill();
        frontRelicGrabber.setPosition(frontRelicGrabber.getPosition());
        backRelicGrabber.setPosition(backRelicGrabber.getPosition());
        return 0;
    }
}
