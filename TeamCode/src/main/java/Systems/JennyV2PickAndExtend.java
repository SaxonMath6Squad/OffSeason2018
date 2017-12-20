package Systems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import Actions.ArialDepositor;
import Actions.WheelPickerDoubleMotor;
import Autonomous.RelicRecoveryField;


/**
 * Created by Jeremy on 8/23/2017.
 */

public class JennyV2PickAndExtend {
    ArialDepositor glyphPlacement;
    WheelPickerDoubleMotor glyphGrabber;
    HardwareMap hardwareMap;

    public JennyV2PickAndExtend(HardwareMap hw) throws Exception{
        hardwareMap = hw;
        glyphGrabber = new WheelPickerDoubleMotor(hardwareMap);
        glyphPlacement = new ArialDepositor(hardwareMap);
    }

    public int liftToPosition(double inInches){
        glyphPlacement.goToLiftPosition(inInches);
        return 0;
    }

    public int lift(){
        glyphPlacement.extend();
        return 0;
    }

    public int drop(){
        glyphPlacement.retract();
        return 0;
    }

    public int pauseLift(){
        glyphPlacement.stopLift();
        return 0;
    }

    public int startGlyphBelt(){
        glyphPlacement.startBelt();
        return 0;
    }

    public int reverseGlyphBelt(){
        glyphPlacement.reverseBelt();
        return 0;
    }

    public int pauseBelt(){
        glyphPlacement.stopBelt();
        return 0;
    }

    public int grab(){
        glyphGrabber.pick();
        return 0;
    }

    public int spit(){
        glyphGrabber.spit();
        return 0;
    }

    public int pauseGrabber(){
        glyphGrabber.pause();
        return 0;
    }

    public int getCurrentTick(int motor){
        switch (motor){
            case RelicRecoveryField.LIFT_MOTOR:
                return (int)glyphPlacement.getMotorPosition();
            default:
                break;
        }
        return 0;
    }

    public int stop(){
        glyphGrabber.pause();
        glyphGrabber.stop();
        glyphPlacement.stop();
        return 0;
    }
}
