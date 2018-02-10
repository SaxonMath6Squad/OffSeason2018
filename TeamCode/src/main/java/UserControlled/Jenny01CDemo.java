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

import android.graphics.Bitmap;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;

import Actions.HardwareWrappers.JennyFlagControllerOneArm;
import Actions.JennyO1BGlyphPicker;
import Actions.JennyO1CRAD;
import Actions.JewelJousterV2;
import Actions.NewArialDepositor;
import Autonomous.ImageAlignmentHelper;
import Autonomous.ImageProcessing.CryptoBoxColumnImageProcessor;
import Autonomous.VuforiaHelper;
import DriveEngine.JennyNavigation;
import SensorHandlers.JennySensorTelemetry;

import static Autonomous.ImageProcessing.CryptoBoxColumnImageProcessor.CLOSE_UP_MIN_COLUMN_WIDTH;
import static Autonomous.ImageProcessing.CryptoBoxColumnImageProcessor.CLOSE_UP_MIN_PERCENT_COLUMN_CHECK;
import static Autonomous.ImageProcessing.CryptoBoxColumnImageProcessor.DESIRED_HEIGHT;
import static Autonomous.ImageProcessing.CryptoBoxColumnImageProcessor.DESIRED_WIDTH;
import static Autonomous.RelicRecoveryField.BLUE_ALLIANCE_2;
import static Autonomous.RelicRecoveryField.CRYPTOBOX_SCORING_COLUMN_WIDTHS_INCHES;
import static Autonomous.RelicRecoveryField.startLocations;
import static DriveEngine.JennyNavigation.DEFAULT_SLEEP_DELAY_MILLIS;
import static DriveEngine.JennyNavigation.EAST;
import static DriveEngine.JennyNavigation.HIGH_SPEED_IN_PER_SEC;
import static DriveEngine.JennyNavigation.NORTH;
import static DriveEngine.JennyNavigation.SLOW_SPEED_IN_PER_SEC;
import static DriveEngine.JennyNavigation.WEST;

/*
    An opmode for the User Controlled portion of the game
    This code was used for the first regional qualifier at Asheville NC
    Limited autonomous, no ciphers on red team, glyphs close but not always scored.
    We won 1st place
 */
@TeleOp(name="Jenny O1C Demo", group="Demos")  // @Autonomous(...) is the other common choice
//@Disabled
public class Jenny01CDemo extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    JoystickHandler leftJoystick, rightJoystick;
    NewArialDepositor glyphLift;
    JennyO1BGlyphPicker glyphPicker;
    JennyO1CRAD RAD;
    JewelJousterV2 jouster;
    JennySensorTelemetry sensorTelemetry;
    JennyFlagControllerOneArm flag;
    boolean autoLiftPositionMode = false;
    CryptoBoxColumnImageProcessor.CRYPTOBOX_COLOR color = CryptoBoxColumnImageProcessor.CRYPTOBOX_COLOR.BLUE;

    boolean radMode = false;
    boolean isFlagOn = false;

    @Override
    public void runOpMode() {
        try{
            glyphLift = new NewArialDepositor(hardwareMap);

        }
        catch(Exception e){
            Log.e("Error!","Glyph Lift: " + e.toString());
            throw new RuntimeException("Glyph Lift Creation Error! " + e.toString());
        }
        try{
            RAD = new JennyO1CRAD(hardwareMap);
        } catch (Exception e){
            Log.e("Error!", "Jenny RAD: " + e.toString());
            throw new RuntimeException("RAD Creation Error! " + e.toString());
        }
        try{
            jouster = new JewelJousterV2("jewelJoust", "jewelJoustTurn", this, hardwareMap);
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
        flag = new JennyFlagControllerOneArm(hardwareMap);
        leftJoystick = new JoystickHandler(gamepad1,JoystickHandler.LEFT_JOYSTICK);
        rightJoystick = new JoystickHandler(gamepad1,JoystickHandler.RIGHT_JOYSTICK);
        jouster.setJoustMode(JewelJousterV2.JEWEL_JOUSTER_POSITIONS.STORE);
        while (!opModeIsActive()) {
            if (gamepad1.start) {
                if(color == CryptoBoxColumnImageProcessor.CRYPTOBOX_COLOR.BLUE) color = CryptoBoxColumnImageProcessor.CRYPTOBOX_COLOR.RED;
                else color = CryptoBoxColumnImageProcessor.CRYPTOBOX_COLOR.BLUE;
                while (gamepad1.start) ;
            }
            telemetry.addData("Color", color);
            telemetry.update();
        }
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        long loopStartTime = 0;

        while (opModeIsActive()) {
            loopStartTime = System.currentTimeMillis();
            jouster.setJoustMode(JewelJousterV2.JEWEL_JOUSTER_POSITIONS.STORE_WITHOUT_PAUSE);
            handlePickSystem();
            handleAerialLift();
            if(radMode) handleRAD();

            if(gamepad1.y && !radMode) {
                autoLiftPositionMode = !autoLiftPositionMode;
                while (gamepad1.y);
            }
            //telemetry.addData("Glyph Lift Time", "" + (System.currentTimeMillis() - glyphLiftStart));
            //GLYPH ROLLER



            //MISC
            if(gamepad1.left_stick_button && gamepad1.right_stick_button) {
                radMode = !radMode;
                while (gamepad1.left_stick_button && gamepad1.right_stick_button);
            }


            if(gamepad1.dpad_down){
                isFlagOn = !isFlagOn;
                if(isFlagOn) flag.startFlag();
                else flag.stopFlag();
                while (gamepad1.dpad_down);
            }

            //jouster.setJoustMode(JewelJousterV2.JEWEL_JOUSTER_POSITIONS.STORE);
            telemetry.addData("lift tick", glyphLift.getLiftMotorPosition());
            telemetry.addData("lift inch",glyphLift.getExtendotronHeight());
            //telemetry.addData("Belt power", glyphLift.getBeltPower());
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            //telemetry.addData("Min Button","" + glyphLift.isPressed());
            telemetry.addData("Time for Loop","" + (System.currentTimeMillis() - loopStartTime));
            telemetry.update();
        }
        glyphLift.kill();
        RAD.kill();
        sensorTelemetry.stopTelemetryLogging();
        glyphPicker.kill();
        flag.killFlag();
        jouster.kill();
    }
    int position = 0;

    public void handleAerialLift(){
        if(gamepad1.dpad_up && autoLiftPositionMode){
            position ++;
            if(position > 3) position = 3;
            switch (position){
                case(1):
                    glyphLift.goToGlyphLevel(NewArialDepositor.GLYPH_PLACEMENT_LEVEL.ROW1_AND_2);
                    break;
                case(2):
                    glyphLift.goToGlyphLevel(NewArialDepositor.GLYPH_PLACEMENT_LEVEL.ROW3);
                    break;
                case(3):
                    glyphLift.goToGlyphLevel(NewArialDepositor.GLYPH_PLACEMENT_LEVEL.ROW4);
                    break;
            }
            while(gamepad1.dpad_up);
        }
        else if(gamepad1.dpad_down && autoLiftPositionMode){
            position --;
            if(position < 0) position = 0;
            switch (position){
                case(1):
                    glyphLift.goToGlyphLevel(NewArialDepositor.GLYPH_PLACEMENT_LEVEL.ROW1_AND_2);
                    break;
                case(2):
                    glyphLift.goToGlyphLevel(NewArialDepositor.GLYPH_PLACEMENT_LEVEL.ROW3);
                    break;
            }
            while(gamepad1.dpad_down);
        }
        if (gamepad1.right_trigger > .1) {
            glyphLift.extend();
        } else if (gamepad1.right_bumper && !glyphLift.isPressed()) {
            glyphLift.retract();
        } else
            glyphLift.stopLift();

        if (!glyphLift.isPressed() && !radMode) {
            //if(gamepad1.left_bumper)
            //if (gamepad1.left_trigger > 0.1) glyphLift.startBeltSlow();
//                else if (gamepad1.left_bumper && !gamepad1.right_bumper)
//                    glyphLift.retractBeltSlow();
            if (gamepad1.left_trigger > 0.1)
                glyphLift.startBeltSlow();
            else if (gamepad1.left_bumper )
                glyphLift.retractBeltSlow();
            else
                glyphLift.stopBelt();
        } else if (!radMode){
            //if (gamepad1.left_trigger > 0.1) glyphLift.startBelt();
            //else if (gamepad1.left_bumper && !gamepad1.right_bumper) glyphLift.retractBelt();
            if (gamepad1.left_trigger > 0.1)
                glyphLift.startBelt();
            else if (gamepad1.left_bumper)
                glyphLift.retractBelt();
            else
                glyphLift.stopBelt();
        }
        if(glyphLift.isPressed()) position = 0;
    }

    public void handleRAD() {
        handleAerialLift();
        //Grabbers
        if (gamepad1.x) {
            RAD.grabFrontRelic();

        } else if (gamepad1.y) {
            RAD.releaseFrontRelic();
        }



        //Extender
        if (gamepad1.left_trigger > 0.1) {
            RAD.extendRAD();
        }
        else if (gamepad1.left_bumper) {
            RAD.retractRAD();
        }
        else RAD.pauseRADExtender();


    }

    public void handlePickSystem(){
        //GLYPH GRABBER
        if(gamepad1.a) glyphPicker.grab();
        else if(gamepad1.b) glyphPicker.spit();
        else glyphPicker.pause();
    }

}

