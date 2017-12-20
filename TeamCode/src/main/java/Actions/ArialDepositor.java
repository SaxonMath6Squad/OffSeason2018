package Actions;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import MotorControllers.NewMotorController;

/**
 * Created by robotics on 12/2/17.
 */

public class ArialDepositor implements ActionHandler {
    SpoolMotor leftLiftMotor;
    ServoHandler belt;
    HardwareMap hardwareMap;

    public ArialDepositor(HardwareMap hw) throws Exception{
        hardwareMap = hw;
        leftLiftMotor = new SpoolMotor(new NewMotorController("liftMotor","MotorConfig/FunctionMotors/SpoolMotor.json", hardwareMap),10,10,100,hardwareMap);

        belt = new ServoHandler("belt",hardwareMap);
        belt.setDirection(Servo.Direction.REVERSE);
    }

    public void extend(){
        leftLiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftLiftMotor.extend();
    }

    public void retract(){
        leftLiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftLiftMotor.retract();
    }

    public void stopLift(){
        leftLiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftLiftMotor.pause();
    }

    public void goToLiftPosition(double positionInInches){
        leftLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftLiftMotor.setPosition(positionInInches);
        leftLiftMotor.setPower(0.3);

    }

    public void setLiftDirection(DcMotor.Direction dir){
        leftLiftMotor.setDirection(dir);
    }

    public void setBeltDirection(Servo.Direction dir){
        belt.setDirection(dir);
    }

    public long getMotorPosition(){
        return leftLiftMotor.getPosition();
    }

    public void startBelt(){
        belt.setPosition(.9);
    }

    public void stopBelt(){
        belt.setPosition(.49);
    }

    public void reverseBelt(){
        belt.setPosition(.1);
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
    public void stop() {
        leftLiftMotor.pause();
        leftLiftMotor.stop();
    }

}
