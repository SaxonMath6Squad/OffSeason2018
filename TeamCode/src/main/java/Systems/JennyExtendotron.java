package Systems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import Actions.ServoGrippers;
import Actions.SpoolMotor;
import MotorControllers.NewMotorController;


/**
 * Created by Jeremy on 8/23/2017.
 */

public class JennyExtendotron {
    double minAngles[];
    double maxAngles[];
    double offsets[];
    double extendInPerSec;
    double retractInPerSec;
    double maxExtendInches;
    final int LEFT_SERVO = 0;
    final int RIGHT_SERVO = 1;
    ServoGrippers grippers;
    SpoolMotor motor;

    public JennyExtendotron(HardwareMap hw) throws Exception{
        //initialize driveMotors
        //initialize sensors
        grippers = new ServoGrippers(new Servo[] {hw.servo.get("leftGripper"), hw.servo.get("rightGripper")}, minAngles, maxAngles, offsets);
        motor = new SpoolMotor(new NewMotorController("spoolMotor", "MotorConfig/FunctionMotors/VerticalExtendotronMotor.json", hw),
            extendInPerSec, retractInPerSec, maxExtendInches, hw);
        grippers.setDirection(Servo.Direction.FORWARD, LEFT_SERVO);
    }

    public int grab(int incrementAngle){
        grippers.setAngle(grippers.getAngles()[LEFT_SERVO] + incrementAngle);
        return 0;
    }

    public int release(int decrementAngle){
        grippers.setAngle(grippers.getAngles()[LEFT_SERVO] - decrementAngle);
        return 0;
    }

    public int lift(){
        motor.extend();
        return 0;
    }

    public int drop(){
        motor.retract();
        return 0;
    }

    public int pause(){
        motor.pause();
        return 0;
    }

    public void stop(){
        motor.stop();
    }
}
