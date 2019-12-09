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

package org.firstinspires.ftc.teamcode.unused;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DeviceManager;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.util.Set;

@Disabled
@TeleOp(name="TeleOp", group="Linear Opmode")
public class LastYearTeleop extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeftDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backRightDrive = null;
    private DcMotor strongBoi= null;
    private DcMotor grab = null;
    private DcMotor launcher = null;
    private DcMotor slide = null;
    private Servo teamMarker = null;
    private Servo wall = null;
    private CRServo collection = null;
    private CRServo wheel = null;

    private DeviceManager usb;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        frontLeftDrive  = hardwareMap.get(DcMotor.class, "motorPort03");
        backLeftDrive = hardwareMap.get(DcMotor.class, "motorPort01");
        frontRightDrive = hardwareMap.get(DcMotor.class, "motorPort02");
        backRightDrive = hardwareMap.get(DcMotor.class, "motorPort00");


        teamMarker = hardwareMap.get(Servo.class, "servoPort00");
        wall = hardwareMap.get(Servo.class, "servoPort01");
        collection = hardwareMap.get(CRServo.class, "servoPort02");
        wheel = hardwareMap.get(CRServo.class, "servoPort03");



        strongBoi = hardwareMap.get(DcMotor.class, "motor2Port02");
        grab = hardwareMap.get(DcMotor.class, "motor2Port03");
        launcher = hardwareMap.get(DcMotor.class, "motor2Port01");
        slide = hardwareMap.get(DcMotor.class, "motor2Port00");



        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        backLeftDrive.setDirection(DcMotor.Direction.FORWARD);

        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);

        collection.setDirection(DcMotorSimple.Direction.FORWARD);
        wheel.setDirection(DcMotorSimple.Direction.FORWARD);




        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            //controller 1
            double LStickY = -gamepad1.left_stick_y;
            double LStickX  =  -gamepad1.left_stick_x;
            double RStickY = -gamepad1.right_stick_y;
            double RStickX = gamepad1.right_stick_x;

            float LTrigger1 = gamepad1.left_trigger;
            float RTrigger1 = gamepad1.right_trigger;

            boolean A1 = gamepad1.a;
            boolean B1 = gamepad1.b;
            boolean X1 = gamepad1.x;
            boolean Y1 = gamepad1.y;

            boolean leftBump1 = gamepad1.left_bumper;
            boolean rightBump1 = gamepad1.right_bumper;



            //controller 2
            double LStickY2 = gamepad2.left_stick_y;
            double RStickY2 = gamepad2.right_stick_y;

            float RTrigger2 = gamepad2.right_trigger;
            float LTrigger2 = gamepad2.left_trigger;

            boolean dUp2 = gamepad2.dpad_up;
            boolean dDown2 = gamepad2.dpad_down;
            boolean dLeft2 = gamepad2.dpad_left;
            boolean dRight2 = gamepad2.dpad_right;

            boolean A2 = gamepad2.a;
            boolean B2 = gamepad2.b;
            boolean X2 = gamepad2.x;
            boolean Y2 = gamepad2.y;

            boolean leftBump = gamepad2.left_bumper;
            boolean rightBump = gamepad2.right_bumper;

            boolean LStickButt = gamepad2.left_stick_button;

            //controller 2/////////////////////////////////////////

            //pulley
            if (Math.abs(LStickY2) > 0) {
                strongBoi.setPower(LStickY2);
                if(LStickY2 == 0) {
                    strongBoi.setPower(0);
                }
            }
            else {
                strongBoi.setPower(0);
            }

            //grabber
            if (Math.abs(RStickY2) > 0) {
                grab.setPower(RStickY2 / 2);
                if (RStickY2 == 0) {
                    grab.setPower(0);
                }
            }
            else {
                grab.setPower(0);
            }



            if (B2) {

            }

            //collection
            if (X2) {
                collection.setPower(1);
            }
            else if(Y2) {
                collection.setPower(0);
            }
            else if (LStickButt) {
                collection.setPower(-1);
            }

            //great wall
            if (rightBump) {
                wall.setPosition(1);
            }
            else if (leftBump) {
                wall.setPosition(.05);
            }


            //controller 1///////////////////////////////////

            //drive
            if (Math.abs(LStickY) > 0) {
                SetPower(LStickY, LStickY, LStickY, LStickY);
            }
            else if (Math.abs(LStickX) > Math.abs(LStickY)) {
                double power = Math.abs(LStickX);
                if (LStickX > 0) {
                    SetPower(power, power/3, power/4.5, power/4.5); //left
                }
                else if (LStickX < 0) {
                    SetPower(power/4.5, power/4.5, power, power/3); //right
                }
            }
            else if(Math.abs(RStickX) > 0) {
                SetPower(-RStickX, -RStickX, RStickX, RStickX);
            }
            else if (Math.abs(LTrigger1) > 0) {
                SetPower(LTrigger1, -LTrigger1, -LTrigger1, LTrigger1);
                telemetry.addData("strafe left", LTrigger1);
            }
            else if (Math.abs(RTrigger1) > 0) {
                SetPower(-RTrigger1, RTrigger1, RTrigger1, -RTrigger1);
                telemetry.addData("strafe right", RTrigger1);
            }
            else {
                SetPower(0, 0, 0, 0);
            }

            //marker
            if (X1) {
                teamMarker.setPosition(1);
            }
            if (Y1) {
                teamMarker.setPosition(0);
            }

            if (rightBump1) {
                wheel.setPower(.5);
            }
            else if (leftBump1) {
                wheel.setPower(0);
            }




            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    }

    public void SetPower(double LFPower, double LBPower, double RFPower, double RBPower) {
        frontLeftDrive.setPower(LFPower);
        backLeftDrive.setPower(LBPower);
        frontRightDrive.setPower(RFPower);
        backRightDrive.setPower(RBPower);
    }
}
