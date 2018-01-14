package Actions;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import MotorControllers.MotorController;


/**
 * Created by Jeremy on 8/23/2017.
 */

/*
    A class to handle our RAD for the relic - Relic Arm Device
 */
public class JennyO1BRAD {
    ServoHandler RADGrabber;
    SpoolMotor RADExtender;
    HardwareMap hardwareMap;
    final double EXTEND = 95;
    final double NEUTRAL = 90;
    final double RETRACT = 60;

    public JennyO1BRAD(HardwareMap hw) throws InterruptedException{
        hardwareMap = hw;
        try {
            RADExtender =
                    new SpoolMotor(new MotorController("radExtender", "MotorConfig/FunctionMotors/RADExtender.json", hardwareMap),
                    10, 10, 100, hardwareMap);
            RADExtender.setDirection(DcMotorSimple.Direction.REVERSE);
            RADGrabber = new ServoHandler("radGrabber", hardwareMap);
        } catch (Exception e){
            throw new RuntimeException(e.toString());
        }
        RADGrabber.setDirection(Servo.Direction.FORWARD);
        RADGrabber.setServoRanges(RETRACT, EXTEND);
        RADGrabber.setDegree(NEUTRAL);
        RADExtender.setDirection(DcMotorSimple.Direction.REVERSE);
        RADExtender.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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

    public int grabRelic(){
        RADGrabber.setDegree(RETRACT);
        return 0;
    }

    public int releaseRelic(){
        RADGrabber.setDegree(EXTEND);
        return 0;
    }

    public int stopRelic(){
        RADGrabber.setDegree(NEUTRAL);
        return 0;
    }

    public int setGrabberPosition(double positionInDeg){
        RADGrabber.setDegree(positionInDeg);
        return 0;
    }

    public int stop(){
        RADExtender.stop();
        RADGrabber.setPosition(RADGrabber.getPosition());
        return 0;
    }
}
