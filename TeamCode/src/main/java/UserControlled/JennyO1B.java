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

import Actions.ArialDepositor;
import Actions.JennyO1BGlyphPicker;
import Actions.JewelJouster;
import DriveEngine.JennyNavigation;
import SensorHandlers.JennySensorTelemetry;
import Actions.JennyO1BRAD;

import static Actions.ArialDepositor.GROUND_LEVEL_PLACEMENT_HEIGHT;
import static Actions.ArialDepositor.ROW1_PLACEMENT_HEIGHT;
import static Actions.ArialDepositor.ROW2_PLACEMENT_HEIGHT;
import static Actions.ArialDepositor.ROW3_PLACEMENT_HEIGHT;
import static Actions.ArialDepositor.ROW4_PLACEMENT_HEIGHT;
import static Autonomous.RelicRecoveryField.BLUE_ALLIANCE_2;
import static Autonomous.RelicRecoveryField.startLocations;
import static DriveEngine.JennyNavigation.NORTH;
import static SensorHandlers.JennySensorTelemetry.RAD_LIMIT;

/*
    An opmode for the User Controlled portion of the game
    This code was used for the first regional qualifier at Asheville NC
    Limited autonomous, no ciphers on red team, glyphs close but not always scored.
    We won 1st place
 */
@TeleOp(name="Jenny O1B User Controlled", group="Linear Opmode")  // @Autonomous(...) is the other common choice
//@Disabled
public class JennyO1B extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    JennyNavigation navigation;
    JoystickHandler leftJoystick, rightJoystick;
    ArialDepositor glyphLift;
    JennyO1BGlyphPicker glyphPicker;
    JennyO1BRAD RAD;
    JewelJouster jouster;
    JennySensorTelemetry sensorTelemetry;
    boolean autoLiftPositionMode = false;
    double[] liftPosition = new double[]{GROUND_LEVEL_PLACEMENT_HEIGHT, ROW1_PLACEMENT_HEIGHT, ROW2_PLACEMENT_HEIGHT, ROW3_PLACEMENT_HEIGHT, ROW4_PLACEMENT_HEIGHT};

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
            glyphLift = new ArialDepositor(hardwareMap);

        }
        catch(Exception e){
            Log.e("Error!","Glyph Lift: " + e.toString());
            throw new RuntimeException("Glyph Lift Creation Error! " + e.toString());
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
        try {
            glyphPicker = new JennyO1BGlyphPicker(hardwareMap);
        } catch (Exception e) {
            Log.e("Error!", "Glyph Picker: " + e.toString());
            throw new RuntimeException("Glyph Picker! " + e.toString());
        }
        sensorTelemetry = new JennySensorTelemetry(hardwareMap, 0, 0);
        leftJoystick = new JoystickHandler(gamepad1,JoystickHandler.LEFT_JOYSTICK);
        rightJoystick = new JoystickHandler(gamepad1,JoystickHandler.RIGHT_JOYSTICK);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        jouster.setPosition(JewelJouster.EXTENDION_MODE.STORE);
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        jouster.setPosition(JewelJouster.EXTENDION_MODE.NEUTRAL);
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
            if(gamepad1.a) glyphPicker.grab();
            else if(gamepad1.b) glyphPicker.spit();
            else if(!gamepad1.a && !gamepad1.b && gamepad2.a) glyphPicker.grab();
            else if(!gamepad1.a && !gamepad1.b && gamepad2.b) glyphPicker.spit();
            else glyphPicker.pause();

            //GLYPH LIFT
            if (!autoLiftPositionMode) {
                if (gamepad1.right_trigger > 0.1) {
                    if(!isSlowMode){
                        glyphLift.extend();
                    }
                    else{
                        glyphLift.slowExtend();
                    }
                }
                else if (gamepad1.right_bumper && !sensorTelemetry.isPressed(sensorTelemetry.EXTEND_LIMIT)){
                    if(!isSlowMode){
                        glyphLift.retract();
                    }
                    else{
                        glyphLift.slowRetract();
                    }
                }
                else if (gamepad2.right_trigger > 0.1 && gamepad1.right_trigger < 0.1 && gamepad1.left_trigger < 0.1 && !gamepad1.right_bumper && !gamepad1.left_bumper){
                    if(!isSlowMode){
                        glyphLift.extend();
                    }
                    else {
                        glyphLift.slowExtend();
                    }
                }

                else if (gamepad2.right_bumper && gamepad1.right_trigger < 0.1 && gamepad1.left_trigger < 0.1 && !gamepad1.right_bumper && !gamepad1.left_bumper && !sensorTelemetry.isPressed(sensorTelemetry.EXTEND_LIMIT)){
                    if(!isSlowMode){
                        glyphLift.retract();
                    }
                    else{
                        glyphLift.slowRetract();
                    }
                }
                else if(!sensorTelemetry.isPressed(sensorTelemetry.EXTEND_LIMIT)) {
                    glyphLift.stopLift();
                }
                else
                    glyphLift.setLiftPower(0);
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
                glyphLift.goToLiftPosition(liftPosition[position]);
            }
            if(gamepad1.back || gamepad2.back) {
                autoLiftPositionMode = (autoLiftPositionMode)? false:true;
                while (gamepad1.back || gamepad2.back);
            }

            //GLYPH ROLLER
            if(!sensorTelemetry.isPressed(sensorTelemetry.EXTEND_LIMIT)) {
                glyphLift.setBeltPower(.25);
                if (gamepad1.left_trigger > 0.1) glyphLift.startBelt();
                else if (gamepad1.left_bumper) glyphLift.reverseBelt();
                else if (gamepad2.left_trigger > 0.1 && gamepad1.right_trigger < 0.1 && gamepad1.left_trigger < 0.1 && !gamepad1.right_bumper && !gamepad1.left_bumper)
                    glyphLift.startBelt();
                else if (gamepad2.left_bumper && gamepad1.right_trigger < 0.1 && gamepad1.left_trigger < 0.1 && !gamepad1.right_bumper && !gamepad1.left_bumper)
                    glyphLift.reverseBelt();
                else
                    glyphLift.stopBelt();
            } else {
                glyphLift.setBeltPower(1);
                if (gamepad1.left_trigger > 0.1) glyphLift.startBelt();
                else if (gamepad1.left_bumper) glyphLift.reverseBelt();
                else if (gamepad2.left_trigger > 0.1 && gamepad1.right_trigger < 0.1 && gamepad1.left_trigger < 0.1 && !gamepad1.right_bumper && !gamepad1.left_bumper)
                    glyphLift.startBelt();
                else if (gamepad2.left_bumper && gamepad1.right_trigger < 0.1 && gamepad1.left_trigger < 0.1 && !gamepad1.right_bumper && !gamepad1.left_bumper)
                    glyphLift.reverseBelt();
                else
                    glyphLift.stopBelt();
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
            else if(gamepad2.dpad_up && !gamepad1.x && !gamepad1.y){
                RAD.grabRelic();
            }
            else if(gamepad2.dpad_down && !gamepad1.x && !gamepad1.y){
                RAD.releaseRelic();
            }
            else {
                RAD.stopRelic();
            }

            if(gamepad1.right_stick_button){
                navigation.turnToHeading(NORTH, this);
            }
            if(gamepad1.left_stick_button){
                navigation.setOrientationOffset(0);
            }

            jouster.setPosition(JewelJouster.EXTENDION_MODE.NEUTRAL);
            telemetry.addData("lift tick", glyphLift.getLiftMotorPosition());
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
        navigation.stopNavigation();
        glyphLift.stop();
        RAD.stop();
        sensorTelemetry.stopTelemetryLogging();
    }
}
