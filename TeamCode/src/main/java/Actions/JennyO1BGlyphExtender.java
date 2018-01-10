package Actions;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import Actions.ArialDepositor;
import Actions.JennyO1BGlyphPicker;
import Autonomous.REVColorDistanceSensorController;
import Autonomous.REVColorDistanceSensorController.color;

import static Autonomous.REVColorDistanceSensorController.type.GLYPH_STACK_O_TRON;


/**
 * Created by Jeremy on 8/23/2017.
 */

/*
    A class to handle our extendotron and glyph picker wheels
 */
public class JennyO1BGlyphExtender {
    ArialDepositor glyphPlacement;
    REVColorDistanceSensorController glyphColorController;
    public final static double GROUND = 0;
    public final static double ROW1 = 6;
    public final static double ROW2 = 11.5;
    public final static double ROW3 = 17.75;
    public final static double ROW4 = 19.25;
    HardwareMap hardwareMap;

    public JennyO1BGlyphExtender(HardwareMap hw) throws Exception{
        hardwareMap = hw;
        glyphPlacement = new ArialDepositor(hardwareMap);
        glyphPlacement.setBeltDirection(DcMotor.Direction.FORWARD);
        glyphPlacement.setLiftDirection(DcMotorSimple.Direction.FORWARD);
        glyphColorController = new REVColorDistanceSensorController(GLYPH_STACK_O_TRON, "glyphSensor", hardwareMap);
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

    public color getColor(){
        return glyphColorController.getColor();
    }

    public double getDistance() {
        return glyphColorController.getDistance(DistanceUnit.CM);
    }

    public int stop(){
        glyphPlacement.stop();
        return 0;
    }
}
