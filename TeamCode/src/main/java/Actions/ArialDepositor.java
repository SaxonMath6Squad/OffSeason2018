package Actions;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import MotorControllers.NewMotorController;

/**
 * Created by robotics on 12/2/17.
 */

public class ArialDepositor implements ActionHandler {
    SpoolMotor leftLiftMotor;
    SpoolMotor belt;
    HardwareMap hardwareMap;

    public ArialDepositor(HardwareMap hw) throws Exception{
        hardwareMap = hw;
        leftLiftMotor = new SpoolMotor(new NewMotorController("liftMotor","MotorConfig/FunctionMotors/SpoolMotor.json", hardwareMap),10,50,100, hardwareMap);

        belt = new SpoolMotor(new NewMotorController("belt", "MotorConfig/FunctionMotors/BeltMotor.json", hardwareMap), 10, 10, 100, hardwareMap);
        belt.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        belt.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void extend(){
        leftLiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftLiftMotor.extend();
    }

    public void retract(){
        leftLiftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftLiftMotor.setPower(-1);
    }

    public void stopLift(){
        if(leftLiftMotor.getMotorControllerMode() == DcMotor.RunMode.RUN_USING_ENCODER || leftLiftMotor.getMotorControllerMode() == DcMotor.RunMode.RUN_WITHOUT_ENCODER) {
            leftLiftMotor.pause();
            leftLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftLiftMotor.setPosition(leftLiftMotor.getPosition());
            leftLiftMotor.setPower(0.2);
        }
    }

    public void goToLiftPosition(double positionInInches){
        leftLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftLiftMotor.setPosition(positionInInches);
        leftLiftMotor.setPower(0.3);

    }

    public void setLiftDirection(DcMotor.Direction dir){
        leftLiftMotor.setDirection(dir);
    }

    public void setBeltDirection(DcMotor.Direction dir){
        belt.setDirection(dir);
    }

    public void setBeltPower(double power){
        belt.setExtendPower(power);
    }

    public long getLiftMotorPosition(){
        return leftLiftMotor.getPosition();
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
