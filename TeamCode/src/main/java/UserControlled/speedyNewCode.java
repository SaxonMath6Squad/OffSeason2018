package UserControlled;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import DriveEngine.HolonomicDriveSystem;

/**
 * Created by robotics on 2/16/18.
 */
@TeleOp(name="New Speedy Tester", group="Linear Opmode")  // @Autonomous(...) is the other common choice
public class speedyNewCode extends LinearOpMode {

    final double movementScale = 1;
    final double turningScale = .5;

    @Override
    public void runOpMode() throws InterruptedException {
        HolonomicDriveSystem driveSystem = new HolonomicDriveSystem(hardwareMap,"RobotConfig/JennyV2.json");
        JoystickHandler leftStick = new JoystickHandler(gamepad1, JoystickHandler.LEFT_JOYSTICK);
        JoystickHandler rightStick = new JoystickHandler(gamepad1, JoystickHandler.RIGHT_JOYSTICK);
        waitForStart();
        double movementPower = 0;
        double turningPower = 0;

        while(opModeIsActive()){
             movementPower = movementScale * leftStick.magnitude();
             turningPower = turningScale * Math.abs(rightStick.magnitude()) * Math.abs(rightStick.x())/rightStick.x();
            driveSystem.driveOnHeadingWithTurning(leftStick.angle(), Math.abs(leftStick.magnitude()), turningPower);
            telemetry.addData("Gamepad1 left Joystick",leftStick.toString());
            telemetry.addData("Gamepad1 right Joystick", rightStick.toString());
            telemetry.update();
        }
    }

}
