package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.CRServo;



import static org.firstinspires.ftc.teamcode.Robot.*;

@TeleOp(name = "testop")
public class Teleop extends LinearOpMode {


    int slidepos = 0;
    int armpos = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        initMotors(this);

        waitForStart();


        //servo starting positions
        grableft.setPosition(.9);
        grabright.setPosition(0);

        while(opModeIsActive()) {
            boolean a1 = gamepad1.a;
            boolean b1 = gamepad1.b;
            boolean x1 = gamepad1.x;
            boolean y1 = gamepad1.y;
            boolean leftBump1 = gamepad1.left_bumper;
            boolean rightBump1 = gamepad1.right_bumper;

            double LStickY = gamepad1.left_stick_y;
            double LStickX  =  -gamepad1.left_stick_x;
            double RStickY = -gamepad1.right_stick_y;
            double RStickX = -gamepad1.right_stick_x;

            double LTrigger1 = -gamepad1.left_trigger;
            double RTrigger1 = -gamepad1.right_trigger;


            boolean a2 = gamepad2.a;
            boolean b2 = gamepad2.b;
            boolean x2 = gamepad2.x;
            boolean y2 = gamepad2.y;

            double LTrigger2 = gamepad2.left_trigger;
            double RTrigger2 = gamepad2.right_trigger;
            boolean LBumper2 = gamepad2.left_bumper;
            boolean RBumper2 = gamepad2.right_bumper;

            double RStickY2 = -gamepad2.right_stick_y;
            double RStickX2 = gamepad2.right_stick_x;
            double LStickY2 = -gamepad2.left_stick_y;

            boolean dpadUp1 = gamepad1.dpad_up;
            boolean dpadRight1 = gamepad1.dpad_right;
            boolean dpadLeft1 = gamepad1.dpad_left;
            boolean dpadDown1 = gamepad1.dpad_down;

            boolean dpadUP2 = gamepad2.dpad_up;
            boolean dpadDOWN2 =gamepad2.dpad_down;
            boolean dpadRight2 = gamepad2.dpad_right;
            boolean dpadLeft2 = gamepad2.dpad_left;



            //locking mechanisms
            if (armboi.getCurrentPosition() > armpos || armboi.getCurrentPosition() < armpos) {
                armboi.setTargetPosition(armpos);
                armboi.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armboi.setPower(1);
                telemetry.addLine("should be moving");
            }
            if (slide.getCurrentPosition() > slidepos || slide.getCurrentPosition() < slidepos) {
                slide.setTargetPosition(slidepos);
                slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slide.setPower(1);
            }



            //diagonal driving
            if (Math.abs(LStickX) > 0 || Math.abs(LStickY) > 0 || Math.abs(RStickX) > 0) {
                if (Math.abs(LStickX) < .05 && Math.abs(RStickX) < .05) {
                    SetPower(LStickY, LStickY, LStickY, LStickY);
                }
                else if (Math.abs(LStickY) < .05 && Math.abs(RStickX) < .05) {
                    SetPower(LStickX, -LStickX, -LStickX, LStickX);
                }
                else {
                    double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
                    double robotAngle = Math.atan2(gamepad1.left_stick_y, -gamepad1.left_stick_x) - Math.PI / 4;
                    double rightX = -gamepad1.right_stick_x;

                    double v1 = r * Math.cos(robotAngle) + rightX; //lf
                    double v2 = r * Math.sin(robotAngle) - rightX; //rf
                    double v3 = r * Math.sin(robotAngle) + rightX; //lb
                    double v4 = r * Math.cos(robotAngle) - rightX; //rb

                    SetPower(v1, v2, v3, v4);
                }
            }
            else if (Math.abs(LTrigger1) > 0) {
                SetPower(-.5 * LTrigger1, .5 * LTrigger1, -.5 * LTrigger1, .5 * LTrigger1);
            }
            else if (Math.abs(RTrigger1) > 0) {
                SetPower(.5 * RTrigger1, -.5 * RTrigger1, .5 * RTrigger1, -.5 * RTrigger1);
            }
            else {
                SetPower(0,0,0,0);
            }


/*
            //driving
            if (Math.abs(LStickY) > 0) {
                SetPower(LStickY, LStickY, LStickY, LStickY);
            }
            else if(Math.abs(RStickX) > 0) {
                SetPower(RStickX, -RStickX, RStickX, -RStickX);
            }
            else if (Math.abs(LTrigger1) > 0) {
                SetPower(-LTrigger1, LTrigger1, LTrigger1, -LTrigger1);
            }
            else if (Math.abs(RTrigger1) > 0) {
                SetPower(RTrigger1, -RTrigger1, -RTrigger1, RTrigger1);
            }
            else if (leftBump1) {
                SetPower(.3, -.3, .3, -.3);
            }
            else if (rightBump1) {
                SetPower(-.3, .3, -.3, .3);
            }
            else {
                SetPower(0, 0, 0, 0);
            }
*/

            //d pad fine tuned driving
            if(dpadUp1){
                SetPower(-.3, -.3, -.3, -.3);
            }
            else if(dpadRight1){
                SetPower(-.5, .5, .5, -.5);
            }
            else if(dpadLeft1){
                SetPower(.5, -.5, -.5, .5);
            }
            else if(dpadDown1){
                SetPower(.3, .3, .3, .3);
            }


            //autonomous servo
            if (x1) { //down
                blockgrabBlue.setPosition(0);
            }
            if (y1) { //up (starting)
                blockgrabBlue.setPosition(1);
            }

            if (a1){
                blockgrabRed.setPosition(1);
            }
            if(b1){
                blockgrabRed.setPosition(0);
            }


            //controller 2////////////////////////////////////////////////////////////////////////////////


            //Wheel grabber (Continuous Servos - Green small wheels)
            if (RBumper2){
                wheel1.setDirection(DcMotorSimple.Direction.FORWARD);
                wheel2.setDirection(DcMotorSimple.Direction.REVERSE);
                wheel1.setPower(1);
                wheel2.setPower(1);
            }
            else if (LBumper2){
                wheel1.setDirection(DcMotorSimple.Direction.REVERSE);
                wheel2.setDirection(DcMotorSimple.Direction.FORWARD);
                wheel1.setPower(1);
                wheel2.setPower(1);

            }
            else {
                wheel1.setDirection(DcMotorSimple.Direction.FORWARD);
                wheel2.setDirection(DcMotorSimple.Direction.REVERSE);
                wheel1.setPower(.03);
                wheel2.setPower(.03);
            }



            //End Wheel Grabber Code

            if (LStickY2 > 0) {
                //up
                slidepos += (75 * Math.abs(LStickY2));
            }
            else if (LStickY2 < 0) {
                //down (has bottom limit)
                if (!(slide.getCurrentPosition() < 15)) {
                    slidepos -= (75 * Math.abs(LStickY2));
                }
            }

            if (RStickY2 > 0) {
                //up
                armpos += (10 * Math.abs(RStickY2));
            }
            else if (RStickY2 < 0) {
                //down
                armpos -= (10 * Math.abs(RStickY2));
            }

            if (LTrigger2 > 0 && grableft.getPosition() > .5 && grabright.getPosition() < .5) {
                grableft.setPosition(grableft.getPosition() - (LTrigger2 * .05));
                grabright.setPosition(grabright.getPosition() + (LTrigger2 * .05));
            }

            if (RTrigger2 > 0  && grableft.getPosition() < 1 && grabright.getPosition() > 0) {
                grableft.setPosition(grableft.getPosition() + (RTrigger2 * .05));
                grabright.setPosition(grabright.getPosition() - (RTrigger2 * .05));
            }






            if (dpadRight2){
                bord1.setPosition(0);
                bord2.setPosition(.5);
            }
            if (dpadLeft2){
                bord1.setPosition(.5);
                bord2.setPosition(0);
            }




            if (dpadDOWN2) {
                armpos -= 10;
            }
            if (dpadUP2) {
                armpos += 10;
            }

            if (a2) {
                armboi.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                armpos = 0;
            }

            if (b2) {
                slidepos = 0;
            }


            if (!x2) {
                cap.setPosition(.4);
            }
            else {
                cap.setPosition(.8);
            }




            telemetry.addData("left front", leftfront.getCurrentPosition());
            telemetry.addData("right front", rightfront.getCurrentPosition());
            telemetry.addData("left back", leftback.getCurrentPosition());
            telemetry.addData("right back", rightback.getCurrentPosition());
            telemetry.addData("right servo", grabright.getPosition());
            telemetry.addData("left servo", grableft.getPosition());
            telemetry.addData("arm", armboi.getCurrentPosition());
            //telemetry.addData("slide", slide.getCurrentPosition());
            telemetry.update();
        }

        //threadSlide.interrupt();
        //threadArm.interrupt();

    }



    public void SetPower(double LFPower, double RFPower, double LBPower, double RBPower) {
        leftfront.setPower(LFPower);
        leftback.setPower(LBPower);
        rightfront.setPower(RFPower);
        rightback.setPower(RBPower);
    }

    public void resetMotors() {
        leftfront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightfront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftback.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightback.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftfront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightfront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftback.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightback.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


    }

}


