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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import Actions.ArialDepositor;

/*
    An opmode to test if all our drive wheels are working correctly
 */
@TeleOp(name="Go To Glyph Level Test", group="Linear Opmode")  // @Autonomous(...) is the other common choice
@Disabled
public class GoToGlyphLevelTest extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    ArialDepositor glyphLift;
    @Override
    public void runOpMode() {
        try {
            glyphLift = new ArialDepositor(hardwareMap);
        }
        catch (Exception e){
            Log.e("Error!" , "Glyph Lift: " + e.toString());
            throw new RuntimeException("Glyph Lift Creation Error! " + e.toString());

        }
        ArialDepositor.GLYPH_PLACEMENT_LEVEL[] levels = new ArialDepositor.GLYPH_PLACEMENT_LEVEL[]
                {ArialDepositor.GLYPH_PLACEMENT_LEVEL.GROUND,
                ArialDepositor.GLYPH_PLACEMENT_LEVEL.ROW1_AND_2,
                ArialDepositor.GLYPH_PLACEMENT_LEVEL.ROW3_AND_4};
        int position = 0;

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        while (opModeIsActive()){
            if(gamepad1.right_trigger > 0.1){
                position++;
                while (gamepad1.right_trigger > 0.1);
            } else if(gamepad1.right_bumper){
                position--;
                while (gamepad1.right_bumper);
            }
            if(position < 0) position = 0;
            else if(position > levels.length-1) position = levels.length-1;
            glyphLift.goToGlyphLevel(levels[position]);
            telemetry.addData("Position", position);
            telemetry.addData("Tick", glyphLift.getLiftMotorPosition());
            telemetry.addData("Inch",glyphLift.getGlyphInch());
            telemetry.addData("Offset", glyphLift.getLiftPositionOffsetTicks());
            telemetry.update();
        }

        glyphLift.kill();
    }
}
