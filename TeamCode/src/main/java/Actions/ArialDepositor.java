package Actions;

import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import Actions.HardwareWrappers.SpoolMotor;
import Autonomous.REVColorDistanceSensorController;
import MotorControllers.MotorController;

/**
 * Created by robotics on 12/2/17.
 */

/*
    A class to set up two motors to use our extendotron and belt for our glyph system
 */
public class ArialDepositor implements ActionHandler {
    SpoolMotor liftMotor;
    SpoolMotor belt;
    REVColorDistanceSensorController glyphSensor;
    TouchSensor extendLimit;
    HardwareMap hardwareMap;
    double currentPosition = 0;
    long liftPositionOffsetTicks = 0;
    private final static double FAST_RETRACT_SPEED = 10.0;
    private final static double FAST_EXTEND_SPEED = 15.0;
    private final static double SLOW_RETRACT_SPEED = 1.0;
    private final static double SLOW_EXTEND_SPEED = 5.0;

    public final static int TICKS_PER_REV = 1120;
    public final static double EXTENDOTRON_DIAMETER_INCHES = 2;

    public enum GLYPH_PLACEMENT_LEVEL{GROUND,ROW1,ROW2,ROW3,ROW4,ROW1_AND_2,ROW3_AND_4};
    public final static double GROUND_LEVEL_PLACEMENT_HEIGHT = 0;
    public final static double ROW1_PLACEMENT_HEIGHT = 6;
    public final static double ROW2_PLACEMENT_HEIGHT = 12;
    public final static double ROW3_PLACEMENT_HEIGHT = 18;
    public final static double ROW4_PLACEMENT_HEIGHT = 24;
    public final static double ROW1_AND_2_PLACEMENT_HEIGHT = 14.5;
    public final static double ROW3_AND_4_PLACEMENT_HEIGHT = 27;

    public ArialDepositor(HardwareMap hw) throws Exception{
        hardwareMap = hw;
        liftMotor = new SpoolMotor(new MotorController("liftMotor","MotorConfig/FunctionMotors/AerialLiftSpool.json", hardwareMap),FAST_EXTEND_SPEED,FAST_RETRACT_SPEED,100, hardwareMap);
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        belt = new SpoolMotor(new MotorController("belt", "MotorConfig/FunctionMotors/BeltMotor.json", hardwareMap), 10, 10, 100, hardwareMap);
        belt.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        glyphSensor = new REVColorDistanceSensorController(REVColorDistanceSensorController.type.GLYPH_STACK_O_TRON, "glyphSensor", hardwareMap);
        extendLimit = hardwareMap.touchSensor.get("extendLimit");
//        zeroBedHeight();
    }

    public void extend(){
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotor.setExtendSpeed(FAST_EXTEND_SPEED);
        liftMotor.extend();
    }

    public void retract(){
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotor.setExtendPower(1);
        liftMotor.retractWithPower();
    }

    public void slowRetract(){
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotor.setExtendPower(0.25);
        liftMotor.retractWithPower();
    }
    public void slowExtend(){
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotor.setExtendSpeed(SLOW_EXTEND_SPEED);
        liftMotor.extend();
    }


    public boolean zeroBedHeight() {
        if (extendLimit.isPressed()) {
            liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            liftMotor.setPower(.1);
            while (extendLimit.isPressed());
            long tickLoc = liftMotor.getPosition();
            liftMotor.holdPosition();
            liftPositionOffsetTicks = tickLoc;
            return true;
        } else {
            liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            liftMotor.setPower(-.1);
            while (!extendLimit.isPressed()) ;
            liftMotor.pause();
            liftMotor.setPower(.1);
            while (extendLimit.isPressed()) ;
            long tickLoc = liftMotor.getPosition();
            liftMotor.holdPosition();
            liftPositionOffsetTicks = tickLoc;
            return true;
        }
    }

    public void stopLift(){
        if(liftMotor.getMotorControllerMode() == DcMotor.RunMode.RUN_USING_ENCODER || liftMotor.getMotorControllerMode() == DcMotor.RunMode.RUN_WITHOUT_ENCODER) {
            liftMotor.holdPosition();
//            liftMotor.setPower(0);
//            currentPosition = liftMotor.getPosition();
//            currentPosition = currentPosition/TICKS_PER_REV*(Math.PI*EXTENDOTRON_DIAMETER_INCHES);
//            Log.d("Extendotron", "Update Position");
//            Log.d("Extendotron", "Tick " + Long.toString(liftMotor.getPosition()));
//            Log.d("Extendotron", "Inch " + Double.toString(currentPosition));
        }
        //goToLiftPosition(currentPosition - liftPositionOffsetTicks);
    }

    public void goToLiftPosition(double positionInInches){
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        double liftPositionOffsetInches = (double)liftPositionOffsetTicks/TICKS_PER_REV*Math.PI*EXTENDOTRON_DIAMETER_INCHES;
        long tick = (long)(positionInInches/(Math.PI*EXTENDOTRON_DIAMETER_INCHES)*TICKS_PER_REV);
        Log.d("Offset inch", Double.toString(liftPositionOffsetInches));
        Log.d("Offset tick", Long.toString(liftPositionOffsetTicks));
        Log.d("Desired inch", Double.toString(positionInInches));
        Log.d("Desired tick", Long.toString(tick));
        liftMotor.setPostitionInches(positionInInches + liftPositionOffsetInches);
        liftMotor.setPower(1);
    }

    public double getGlyphInch(){
        return (double) liftMotor.getPosition()/TICKS_PER_REV*Math.PI*EXTENDOTRON_DIAMETER_INCHES;
    }

    public void goToGlyphLevel(GLYPH_PLACEMENT_LEVEL level){
        switch(level){
            case GROUND:
                goToLiftPosition(GROUND_LEVEL_PLACEMENT_HEIGHT);
                break;
            case ROW1:
                goToLiftPosition(ROW1_PLACEMENT_HEIGHT);
                break;
            case ROW2:
                goToLiftPosition(ROW2_PLACEMENT_HEIGHT);
                break;
            case ROW3:
                goToLiftPosition(ROW3_PLACEMENT_HEIGHT);
                break;
            case ROW4:
                goToLiftPosition(ROW4_PLACEMENT_HEIGHT);
                break;
            case ROW1_AND_2:
                goToLiftPosition(ROW1_AND_2_PLACEMENT_HEIGHT);
                break;
            case ROW3_AND_4:
                goToLiftPosition(ROW3_AND_4_PLACEMENT_HEIGHT);
                break;
        }
    }

    public void setLiftPositionOffsetTicks(long offset){
        liftPositionOffsetTicks = offset;
    }

    public void setLiftPower(double power){
        liftMotor.setPower(power);
    }

    public void setLiftDirection(DcMotor.Direction dir){
        liftMotor.setDirection(dir);
    }

    public void setBeltDirection(DcMotor.Direction dir){
        belt.setDirection(dir);
    }

    public void setBeltPower(double power){
        belt.setExtendPower(power);
    }

    public long getLiftPositionOffsetTicks() {
        return liftPositionOffsetTicks;
    }

    public long getLiftMotorPosition(){
        return liftMotor.getPosition();
    }

    public void startBelt(){
        belt.extendWithPower();
    }

    public void stopBelt(){
        belt.pause();
    }

    public void reverseGlyphs(){
        belt.retractWithPower();
    }

    public REVColorDistanceSensorController.color getColor() {
        return glyphSensor.getColor();
    }

    public double getDistance(DistanceUnit unit){
        return glyphSensor.getDistance(unit);
    }

    public boolean isPressed(){
        return extendLimit.isPressed();
    }

    @Override
    public boolean doAction(String action, long maxTimeAllowed) {
        return false;
    }

    @Override
    public boolean stopAction(String action) {
        return false;
    }

    @Override
    public boolean startDoingAction(String action) {
        return false;
    }

    @Override
    public void kill() {
        liftMotor.pause();
        liftMotor.kill();
    }

}
