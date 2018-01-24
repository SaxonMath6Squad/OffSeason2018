package Actions;

import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import Actions.HardwareWrappers.NewSpoolMotor;
import Actions.HardwareWrappers.SpoolMotor;
import Autonomous.REVColorDistanceSensorController;
import MotorControllers.MotorController;

/**
 * Created by robotics on 12/2/17.
 */

/*
    A class to set up two motors to use our extendotron and belt for our glyph system
 */
public class ArialDepositorTest implements ActionHandler {
    NewSpoolMotor liftMotor;
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

    public ArialDepositorTest(HardwareMap hw) throws Exception{
        hardwareMap = hw;
        liftMotor = new NewSpoolMotor("liftMotor","MotorConfig/FunctionMotors/AerialLiftSpool.json", FAST_EXTEND_SPEED, FAST_RETRACT_SPEED, hardwareMap);
        liftMotor.setMotorRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        belt = new SpoolMotor(new MotorController("belt", "MotorConfig/FunctionMotors/BeltMotor.json", hardwareMap), 10, 10, 100, hardwareMap);
        belt.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        glyphSensor = new REVColorDistanceSensorController(REVColorDistanceSensorController.type.GLYPH_STACK_O_TRON, "glyphSensor", hardwareMap);
        extendLimit = hardwareMap.touchSensor.get("extendLimit");
//        zeroBedHeight();
    }

    public void extend(){
        liftMotor.setMotorRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotor.setInchesPerSecondVelocity(FAST_EXTEND_SPEED);
        liftMotor.extend();
    }

    public void retract(){
        liftMotor.setMotorRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotor.setInchesPerSecondVelocity(FAST_RETRACT_SPEED);
        liftMotor.retract();
    }

    public void slowRetract(){
        liftMotor.setMotorRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotor.setInchesPerSecondVelocity(SLOW_RETRACT_SPEED);
        liftMotor.retract();
    }
    public void slowExtend(){
        liftMotor.setMotorRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotor.setInchesPerSecondVelocity(SLOW_EXTEND_SPEED);
        liftMotor.extend();
    }


    public boolean zeroBedHeight() {
        if (extendLimit.isPressed()) {
            liftMotor.setMotorRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
            liftMotor.setMotorPower(.1);
            while (extendLimit.isPressed());
            long tickLoc = liftMotor.getCurrentTick();
            liftMotor.holdPosition();
            liftPositionOffsetTicks = tickLoc;
            return true;
        } else {
            liftMotor.setMotorRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
            liftMotor.setMotorPower(-.1);
            while (!extendLimit.isPressed()) ;
            liftMotor.brake();
            liftMotor.setMotorPower(.1);
            while (extendLimit.isPressed()) ;
            long tickLoc = liftMotor.getCurrentTick();
            liftMotor.holdPosition();
            liftPositionOffsetTicks = tickLoc;
            return true;
        }
    }

    public void stopLift(){
        if(liftMotor.getMotorRunMode() == DcMotor.RunMode.RUN_USING_ENCODER || liftMotor.getMotorRunMode() == DcMotor.RunMode.RUN_WITHOUT_ENCODER) {
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
        liftMotor.setMotorRunMode(DcMotor.RunMode.RUN_TO_POSITION);
        double liftPositionOffsetInches = (double)liftPositionOffsetTicks/TICKS_PER_REV*Math.PI*EXTENDOTRON_DIAMETER_INCHES;
        long tick = (long)(positionInInches/(Math.PI*EXTENDOTRON_DIAMETER_INCHES)*TICKS_PER_REV);
        Log.d("Offset inch", Double.toString(liftPositionOffsetInches));
        Log.d("Offset tick", Long.toString(liftPositionOffsetTicks));
        Log.d("Desired inch", Double.toString(positionInInches));
        Log.d("Desired tick", Long.toString(tick));
        liftMotor.setPositionInches(positionInInches + liftPositionOffsetInches);
        liftMotor.setMotorPower(1);
    }

    public double getGlyphInch(){
        return (double) liftMotor.getCurrentTick()/TICKS_PER_REV*Math.PI*EXTENDOTRON_DIAMETER_INCHES;
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
        liftMotor.setMotorPower(power);
    }

    public void setLiftDirection(DcMotor.Direction dir){
        liftMotor.setMotorDirection(dir);
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
        return liftMotor.getCurrentTick();
    }

    public void startBelt(){
        belt.extendWithPower();
    }

    public void stopBelt(){
        belt.pause();
    }

    public void reverseBelt(){
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

    public double getRPS(){
        return liftMotor.getCurrentRPS();
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
        liftMotor.brake();
        liftMotor.killMotorController();
    }

}