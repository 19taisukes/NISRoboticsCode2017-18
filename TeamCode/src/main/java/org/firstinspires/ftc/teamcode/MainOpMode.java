/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

        /*
            -- Right Bumper --
        •   Extend Recovery Arm Out

            -- Left Bumper --
        •   Extend Recovery Arm In

            -- Right Trigger --
        •   Extend Lift

            -- Left Trigger --
        •   Shorten Lift

            -- X --
        •   Toggle Hand

            -- Y --
        •   Toggle Wrist

            -- A --
        •   Release Grip on Glyph Holder

            -- B --
        •   Toggle Glyph Holder

            -- Right Stick Button --
        •   Toggle Stealth Mode

            -- Right Stick --
        •   Move Robot
        */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Main OpMode", group="Iterative Opmode")
//@Disabled
public class MainOpMode extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor topLeftDrive = null;
    private DcMotor topRightDrive = null;
    private DcMotor bottomLeftDrive = null;
    private DcMotor bottomRightDrive = null;

    private DcMotor lift = null;
    private DcMotor recoveryArm = null;

    private Servo leftGlyph = null;
    private Servo rightGlyph = null;
    private Servo recoveryWrist = null;
    private Servo recoveryHand = null;

    boolean stealthToggle = true;
    boolean glyphToggle = true;
    boolean wristToggle = true;
    boolean handToggle = true;

    boolean isStealth = false;
    boolean isGlyph = false;
    boolean isWrist = true;
    boolean isHand = true;

    double stealth = 1.0;

    double leftGlyphPos = 1.0;
    double rightGlyphPos = 0.0;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init()
    {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        topLeftDrive  = hardwareMap.get(DcMotor.class, "topLeftDrive");
        topRightDrive = hardwareMap.get(DcMotor.class,   "topRightDrive");
        bottomLeftDrive  = hardwareMap.get(DcMotor.class, "bottomLeftDrive");
        bottomRightDrive = hardwareMap.get(DcMotor.class, "bottomRightDrive");

        lift  = hardwareMap.get(DcMotor.class, "lift");
        recoveryArm = hardwareMap.get(DcMotor.class, "recoveryArm");

        leftGlyph = hardwareMap.get(Servo.class,"leftGlyph");
        rightGlyph = hardwareMap.get(Servo.class,"rightGlyph");
        recoveryWrist = hardwareMap.get(Servo.class, "recoveryWrist");
        recoveryHand = hardwareMap.get(Servo.class, "recoveryHand");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        bottomLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        topLeftDrive.setDirection(DcMotor.Direction.REVERSE);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop()
    {

    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start()
    {
        leftGlyph.setPosition(leftGlyphPos);
        rightGlyph.setPosition(rightGlyphPos);

        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop()
    {
        // Setup a variable for each drive wheel to save power level for telemetrY

        if(gamepad1.right_bumper)
        {
            recoveryArm.setPower(-1.0);
        }
        else if(gamepad1.left_bumper)
        {
            recoveryArm.setPower(0.2);
        }
        else
        {
            recoveryArm.setPower(0.0);
        }

        if(gamepad1.y && wristToggle)
        {
            wristToggle = false;
            if(isWrist)
            {
                isWrist = false;
            }
            else if(!isWrist)
            {
                isWrist = true;
            }
        }
        else if(!gamepad1.y)
        {
            wristToggle = true;
        }

        if(gamepad1.x && handToggle)
        {
            handToggle = false;
            if(isHand)
            {
                isHand = false;
            }
            else if(!isHand)
            {
                isHand = true;
            }
        }
        else if(!gamepad1.x)
        {
            handToggle = true;
        }

        lift.setPower(gamepad1.right_trigger - gamepad1.left_trigger);

        if(toggleButton(gamepad1.b, glyphToggle, isGlyph))
        {
            leftGlyphPos = 1.0;
            rightGlyphPos = 0.0;
        }
        else
        {
            leftGlyphPos = 0.0;
            rightGlyphPos = 1.0;
        }

        if(gamepad1.a)
        {
            leftGlyphPos = 0.1;
            rightGlyphPos = 0.9;
        }

        if(gamepad1.right_stick_button && stealthToggle)
        {
            stealthToggle = false;
            if(isStealth)
            {
                isStealth = false;
                stealth = 1.0;
            }
            else if(!isStealth)
            {
                isStealth = true;
                stealth = 0.3;
            }
        }
        else if(!gamepad1.right_stick_button)
        {
            stealthToggle = true;
        }

        double rightPower = (stealth * (gamepad1.right_stick_y + gamepad1.right_stick_x));
        double leftPower = (stealth * (gamepad1.right_stick_y - gamepad1.right_stick_x));

        topLeftDrive.setPower(leftPower);
        topRightDrive.setPower(rightPower);
        bottomLeftDrive.setPower(leftPower);
        bottomRightDrive.setPower(rightPower);

        leftGlyph.setPosition(leftGlyphPos);
        rightGlyph.setPosition(rightGlyphPos);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop()
    {

    }

    public boolean toggleButton(boolean button, boolean toggle, boolean isToggle)
    {
        if(button && toggle)
        {
            toggle = false;
            if(isToggle)
            {
                isToggle = false;
            }
            else if(!isToggle)
            {
                isToggle = true;
            }
        }
        else if(!button)
        {
            toggle = true;
        }
        return isToggle;
    }

}