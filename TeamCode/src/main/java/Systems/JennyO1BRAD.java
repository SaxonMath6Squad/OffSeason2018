package Systems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import Actions.ServoGrippers;
import Actions.ServoHandler;
import Actions.SpoolMotor;
import MotorControllers.NewMotorController;


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

    public JennyO1BRAD(HardwareMap hw) throws InterruptedException{
        hardwareMap = hw;
        try {
            RADExtender =
                    new SpoolMotor(new NewMotorController("radExtender", "MotorConfig/FunctionMotors/RADExtender.json", hardwareMap),
                    10, 10, 100, hardwareMap);
            RADGrabber = new ServoHandler("radGrabber", hardwareMap);
        } catch (Exception e){
            throw new RuntimeException(e.toString());
        }
        RADGrabber.setDirection(Servo.Direction.FORWARD);
        RADExtender.setDirection(DcMotorSimple.Direction.FORWARD);
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
        RADGrabber.incrementDegree(5);
        return 0;
    }

    public int releaseRelic(){
        RADGrabber.incrementDegree(-5);
        return 0;
    }

    public int stop(){
        RADExtender.stop();
        RADGrabber.setPosition(RADGrabber.getPosition());
        return 0;
    }
}
