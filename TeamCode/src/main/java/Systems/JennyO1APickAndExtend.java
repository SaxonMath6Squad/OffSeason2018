package Systems;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import Actions.ArialDepositor;
import Actions.WheelPickerDoubleMotor;
import Actions.WheelPickerSingleMotor;


/**
 * Created by Jeremy on 8/23/2017.
 */

public class JennyO1APickAndExtend {
    ArialDepositor glyphPlacement;
    WheelPickerSingleMotor glyphGrabber;
    HardwareMap hardwareMap;

    public JennyO1APickAndExtend(HardwareMap hw) throws Exception{
        hardwareMap = hw;
        glyphGrabber = new WheelPickerSingleMotor(hardwareMap);
        glyphPlacement = new ArialDepositor(hardwareMap);
        glyphPlacement.setBeltDirection(Servo.Direction.FORWARD);
        glyphPlacement.setLiftDirection(DcMotorSimple.Direction.REVERSE);
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

    public int stop(){
        glyphGrabber.pause();
        glyphGrabber.stop();
        glyphPlacement.stop();
        return 0;
    }
}
