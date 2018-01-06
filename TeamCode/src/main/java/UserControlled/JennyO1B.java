/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package UserControlled;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import Actions.JewelJouster;
import DriveEngine.JennyNavigation;
import SensorHandlers.JennySensorTelemetry;
import Systems.JennyO1BPickAndExtend;
import Systems.JennyO1BRAD;

import static Autonomous.RelicRecoveryField.BLUE_ALLIANCE_2;
import static Autonomous.RelicRecoveryField.GROUND;
import static Autonomous.RelicRecoveryField.ROW1;
import static Autonomous.RelicRecoveryField.ROW2;
import static Autonomous.RelicRecoveryField.ROW3;
import static Autonomous.RelicRecoveryField.ROW4;
import static Autonomous.RelicRecoveryField.startLocations;
import static DriveEngine.JennyNavigation.NORTH;
//import static SensorHandlers.JennySensorTelemetry.JEWEL_JOUST_STORE_POSITION;
import static SensorHandlers.JennySensorTelemetry.RAD_LIMIT;

/*
    An opmode for the User Controlled portion of the game
 */
@TeleOp(name="Jenny O1B User Controlled", group="Linear Opmode")  // @Autonomous(...) is the other common choice
//@Disabled
public class JennyO1B extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    JennyNavigation navigation;
    JoystickHandler leftJoystick, rightJoystick;
    JennyO1BPickAndExtend glyphSystem;
    JennyO1BRAD RAD;
    JewelJouster jouster;
    JennySensorTelemetry sensorTelemetry;
    boolean autoLiftPositionMode = false;
    double[] liftPosition = {GROUND, ROW1, ROW2, ROW3, ROW4};
    int position = 0;

    @Override
    public void runOpMode() {
        try {
            navigation = new JennyNavigation(hardwareMap, startLocations[BLUE_ALLIANCE_2], 0, "RobotConfig/JennyV2.json");
        }
        catch (Exception e){
            Log.e("Error!" , "Jenny Navigation: " + e.toString());
            throw new RuntimeException("Navigation Creation Error! " + e.toString());

        }
        try{
            glyphSystem = new JennyO1BPickAndExtend(hardwareMap);

        }
        catch(Exception e){
            Log.e("Error!","Jenny Wheel Picks: " + e.toString());
            throw new RuntimeException("Wheel Picker Creation Error! " + e.toString());
        }
        try{
            RAD = new JennyO1BRAD(hardwareMap);
        } catch (Exception e){
            Log.e("Error!", "Jenny RAD: " + e.toString());
            throw new RuntimeException("RAD Creation Error! " + e.toString());
        }
        try{
            jouster = new JewelJouster("jewelJouster",hardwareMap);
        } catch (Exception e){
            Log.e("Error!", "Jenny JewelJouster: " + e.toString());
            throw new RuntimeException("Jenny JewelJouster! " + e.toString());
        }
        sensorTelemetry = new JennySensorTelemetry(hardwareMap, 0, 0);
        leftJoystick = new JoystickHandler(gamepad1,JoystickHandler.LEFT_JOYSTICK);
        rightJoystick = new JoystickHandler(gamepad1,JoystickHandler.RIGHT_JOYSTICK);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        boolean isSlowMode = false;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            //DRIVE

            if(rightJoystick.magnitude() < .1){
                if(!isSlowMode)
                navigation.driveOnHeading(leftJoystick.angle(), leftJoystick.magnitude() * 50);
                else navigation.driveOnHeading(leftJoystick.angle(), leftJoystick.magnitude() * 10);
            }
            else {
                if(!isSlowMode) navigation.turn(rightJoystick.magnitude() * rightJoystick.x()/Math.abs(rightJoystick.x()));
                else navigation.turn(.1 * rightJoystick.magnitude() * rightJoystick.x()/Math.abs(rightJoystick.x()));
            }
            if(gamepad1.start){
                isSlowMode = !isSlowMode;
                while(gamepad1.start);
            }

            //GLYPH GRABBER
            if(gamepad1.a) glyphSystem.grab();
            else if(gamepad1.b) glyphSystem.spit();
            else if(!gamepad1.a && !gamepad1.b && gamepad2.a) glyphSystem.grab();
            else if(!gamepad1.a && !gamepad1.b && gamepad2.b) glyphSystem.spit();
            else glyphSystem.pauseGrabber();

            //GLYPH LIFT
            if (!autoLiftPositionMode) {
                if (gamepad1.right_trigger > 0.1) {
                    if(!isSlowMode){
                        glyphSystem.lift();
                    }
                    else{
                        glyphSystem.liftSlow();
                    }
                }
                else if (gamepad1.right_bumper && !sensorTelemetry.isPressed(sensorTelemetry.EXTEND_LIMIT)){
                    if(!isSlowMode){
                        glyphSystem.drop();
                    }
                    else{
                        glyphSystem.dropSlow();
                    }
                }
                else if (gamepad2.right_trigger > 0.1 && gamepad1.right_trigger < 0.1 && gamepad1.left_trigger < 0.1 && !gamepad1.right_bumper && !gamepad1.left_bumper){
                    if(!isSlowMode){
                        glyphSystem.lift();
                    }
                    else {
                        glyphSystem.liftSlow();
                    }
                }

                else if (gamepad2.right_bumper && gamepad1.right_trigger < 0.1 && gamepad1.left_trigger < 0.1 && !gamepad1.right_bumper && !gamepad1.left_bumper && !sensorTelemetry.isPressed(sensorTelemetry.EXTEND_LIMIT)){
                    if(!isSlowMode){
                        glyphSystem.drop();
                    }
                    else{
                        glyphSystem.dropSlow();
                    }
                }
                else if(!sensorTelemetry.isPressed(sensorTelemetry.EXTEND_LIMIT)) {
                    glyphSystem.pauseLift();
                }
                else
                    glyphSystem.setLiftPower(0);
            } else {
                if (gamepad1.right_trigger > 0.1) {
                    position++;
                    while (gamepad1.right_trigger > 0.1) ;
                }
                if (gamepad1.right_bumper) {
                    position--;
                    while (gamepad1.right_bumper) ;
                }
                if (gamepad2.right_trigger > 0.1 && gamepad1.right_trigger < 0.1 && gamepad1.left_trigger < 0.1 && !gamepad1.right_bumper && !gamepad1.left_bumper) {
                    position++;
                    while (gamepad2.right_trigger > 0.1) ;
                }

                if (gamepad2.right_bumper && gamepad1.right_trigger < 0.1 && gamepad1.left_trigger < 0.1 && !gamepad1.right_bumper && !gamepad1.left_bumper) {
                    position--;
                    while (gamepad2.right_bumper) ;
                }

                if (position <= 0) position = 0;
                if (position >= liftPosition.length - 1) position = liftPosition.length - 1;
                glyphSystem.liftToPosition(liftPosition[position]);
            }
            if(gamepad1.back || gamepad2.back) {
                autoLiftPositionMode = (autoLiftPositionMode)? false:true;
                while (gamepad1.back || gamepad2.back);
            }

            //GLYPH ROLLER
            if(!sensorTelemetry.isPressed(sensorTelemetry.EXTEND_LIMIT)) {
                glyphSystem.setBeltPower(.25);
                if (gamepad1.left_trigger > 0.1) glyphSystem.startGlyphBelt();
                else if (gamepad1.left_bumper) glyphSystem.reverseGlyphBelt();
                else if (gamepad2.left_trigger > 0.1 && gamepad1.right_trigger < 0.1 && gamepad1.left_trigger < 0.1 && !gamepad1.right_bumper && !gamepad1.left_bumper)
                    glyphSystem.startGlyphBelt();
                else if (gamepad2.left_bumper && gamepad1.right_trigger < 0.1 && gamepad1.left_trigger < 0.1 && !gamepad1.right_bumper && !gamepad1.left_bumper)
                    glyphSystem.reverseGlyphBelt();
                else
                    glyphSystem.pauseBelt();
            } else {
                glyphSystem.setBeltPower(1);
                if (gamepad1.left_trigger > 0.1) glyphSystem.startGlyphBelt();
                else if (gamepad1.left_bumper) glyphSystem.reverseGlyphBelt();
                else if (gamepad2.left_trigger > 0.1 && gamepad1.right_trigger < 0.1 && gamepad1.left_trigger < 0.1 && !gamepad1.right_bumper && !gamepad1.left_bumper)
                    glyphSystem.startGlyphBelt();
                else if (gamepad2.left_bumper && gamepad1.right_trigger < 0.1 && gamepad1.left_trigger < 0.1 && !gamepad1.right_bumper && !gamepad1.left_bumper)
                    glyphSystem.reverseGlyphBelt();
                else
                    glyphSystem.pauseBelt();
            }

            //RAD Extender
            if(gamepad1.dpad_up){
                RAD.extendRAD();
            }
            else if(gamepad1.dpad_down && !sensorTelemetry.isPressed(RAD_LIMIT)){
                RAD.retractRAD();
            }
            else {
                RAD.pauseRADExtender();
            }

            //RAD Grabber
            if(gamepad1.x){
                RAD.grabRelic();
            }
            else if(gamepad1.y){
                RAD.releaseRelic();
            }

            if(gamepad1.right_stick_button){
                navigation.turnToHeading(NORTH, this);
            }

            //sensorTelemetry.setJewelJoustPosition(JEWEL_JOUST_STORE_POSITION);
            telemetry.addData("lift tick", glyphSystem.getLiftPosition());
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
        navigation.stopNavigation();
        glyphSystem.stop();
        RAD.stop();
        sensorTelemetry.stopTelemetryLogging();
    }
}
