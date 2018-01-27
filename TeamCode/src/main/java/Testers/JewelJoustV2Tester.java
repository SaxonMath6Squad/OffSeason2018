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

import Actions.HardwareWrappers.NewArialDepositor;
import Actions.JewelJousterV2;

/*
    An opmode to test if all our drive wheels are working correctly
 */
@TeleOp(name="Jewel Joust V2 Tester", group="Linear Opmode")  // @Autonomous(...) is the other common choice
//@Disabled
public class JewelJoustV2Tester extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    JewelJousterV2 jewelJoust;

    @Override
    public void runOpMode() {
        try {
            jewelJoust = new JewelJousterV2("jewelJoust", "jewelJoustTurn", this, hardwareMap);
        }
        catch (Exception e){
            Log.e("Error!" , "Glyph Lift: " + e.toString());
            throw new RuntimeException("Glyph Lift Creation Error! " + e.toString());

        }

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        while (opModeIsActive()){
            if(gamepad1.b){
                jewelJoust.setJoustMode(JewelJousterV2.JEWEL_JOUSTER_POSITIONS.STORE);
            }
            else if(gamepad1.a){
                jewelJoust.setJoustMode(JewelJousterV2.JEWEL_JOUSTER_POSITIONS.READ);
            }
            else if(gamepad1.dpad_left){
                jewelJoust.setJoustMode(JewelJousterV2.JEWEL_JOUSTER_POSITIONS.HIT_LEFT);
            }
            else if(gamepad1.dpad_right){
                jewelJoust.setJoustMode(JewelJousterV2.JEWEL_JOUSTER_POSITIONS.HIT_RIGHT);
            }

            telemetry.addData("Color", jewelJoust.getJewelColor());
            telemetry.update();
        }
    }
}
