package Systems;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import Actions.ArialDepositor;
import Actions.WheelPickerSingleMotor;
import Autonomous.ColorModeController;
import Autonomous.ColorModeController.color;

import static Autonomous.ColorModeController.type.GLYPH_STACK_O_TRON;


/**
 * Created by Jeremy on 8/23/2017.
 */

/*
    A class to handle our extendotron and glyph picker wheels
 */
public class JennyO1BPickAndExtend {
    ArialDepositor glyphPlacement;
    WheelPickerSingleMotor glyphGrabber;
    ColorSensor glyphSensor;
    ColorModeController glyphColorController;
    HardwareMap hardwareMap;

    public JennyO1BPickAndExtend(HardwareMap hw) throws Exception{
        hardwareMap = hw;
        glyphGrabber = new WheelPickerSingleMotor(hardwareMap);
        glyphPlacement = new ArialDepositor(hardwareMap);
        glyphPlacement.setBeltDirection(DcMotor.Direction.FORWARD);
        glyphPlacement.setLiftDirection(DcMotorSimple.Direction.FORWARD);
        glyphSensor = hardwareMap.colorSensor.get("glyphSensor");
        glyphColorController = new ColorModeController(GLYPH_STACK_O_TRON, glyphSensor);
    }

    public int liftToPosition(double inInches){
        glyphPlacement.goToLiftPosition(inInches);
        return 0;
    }

    public int lift(){
        glyphPlacement.extend();
        return 0;
    }
    public int liftSlow(){
        glyphPlacement.slowExtend();
        return 0;
    }
    public int dropSlow(){
        glyphPlacement.slowRetract();
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

    public int setLiftPower(double power){
        glyphPlacement.setLiftPower(power);
        return 0;
    }

    public long getLiftPosition(){
        return glyphPlacement.getLiftMotorPosition();
    }

    public int setBeltPower(double power){
        glyphPlacement.setBeltPower(power);
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

    public color getColor(){
        return glyphColorController.getColor();
    }

    public int stop(){
        glyphGrabber.pause();
        glyphGrabber.stop();
        glyphPlacement.stop();
        return 0;
    }
}
