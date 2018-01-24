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
package Testers;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import Actions.ArialDepositorTest;
import DriveEngine.JennyNavigation;

import static Autonomous.RelicRecoveryField.BLUE_ALLIANCE_2;
import static Autonomous.RelicRecoveryField.startLocations;
import static DriveEngine.JennyNavigation.LONG_SLEEP_DELAY_MILLIS;
import static DriveEngine.JennyNavigation.MED_SLEEP_DELAY_MILLIS;

/*
    An opmode to test if all our drive wheels are working correctly
 */
@TeleOp(name="Motor stopper Test", group="Linear Opmode")  // @Autonomous(...) is the other common choice
//@Disabled
public class MotorStopperTest extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    JennyNavigation navigation;
    ArialDepositorTest glyphPlacement;
    private long start;
    private long expectedDelta;
    private long delta = 0;
    private long newPosition = 0;
    @Override
    public void runOpMode() {
        boolean shouldKill = false;
        try {
            navigation = new JennyNavigation(hardwareMap, startLocations[BLUE_ALLIANCE_2], 0, "RobotConfig/JennyV2.json");
            glyphPlacement = new ArialDepositorTest(hardwareMap);
        }
        catch (Exception e){
            Log.e("Error!" , "Jenny Navigation: " + e.toString());
            throw new RuntimeException("Navigation Creation Error! " + e.toString());

        }
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        expectedDelta = 2;
        start = glyphPlacement.getLiftMotorPosition();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        glyphPlacement.setLiftPower(0.5);
        sleep(MED_SLEEP_DELAY_MILLIS + LONG_SLEEP_DELAY_MILLIS);
        while (opModeIsActive() && !shouldKill) {
            glyphPlacement.setLiftPower(0.5);
            newPosition = glyphPlacement.getLiftMotorPosition();
            delta = newPosition - start;
            Log.d("Start Value", "" + start);
            Log.d("New Value", "" + newPosition);
            Log.d("Delta", Long.toString(delta));
            if(delta < expectedDelta){
                shouldKill = true;
            }
            else{
               start = glyphPlacement.getLiftMotorPosition();
            }
        }
        glyphPlacement.stopLift();
        while (!glyphPlacement.isPressed() && opModeIsActive()){
            glyphPlacement.retract();
        }
        glyphPlacement.kill();
        navigation.stopNavigation();
    }
}