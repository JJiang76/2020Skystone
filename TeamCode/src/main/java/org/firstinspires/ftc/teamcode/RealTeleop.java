package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.Robot;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.teamcode.Robot.*;


@TeleOp(name = "teleoperation")
public class RealTeleop extends LinearOpMode {
    
    

    ArrayList<String> info = new ArrayList<>();
    String dir = "";

    int slidepos = 0;
    int armpos = 0;


    @Override
    public void runOpMode() throws InterruptedException {
        initVuforia(this);
        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod(this);
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }
        if (Robot.tfod != null) {
            tfod.activate();
        }

        initMotors(this);

        resetMotors();

        waitForStart();

        //servo starting positions
        grableft.setPosition(.9);
        grabright.setPosition(0);


        double getLeftPos = 0;
        double getRightPos = 0;

        while(opModeIsActive()) {
            boolean a1 = gamepad1.a;
            boolean x1 = gamepad1.x;
            boolean y1 = gamepad1.y;
            boolean leftBump1 = gamepad1.left_bumper;
            boolean rightBump1 = gamepad1.right_bumper;

            double LStickY = gamepad1.left_stick_y / 2;
            double LStickX  =  -gamepad1.left_stick_x;
            double RStickY = -gamepad1.right_stick_y;
            double RStickX = -gamepad1.right_stick_x / 2;

            double LTrigger1 = -gamepad1.left_trigger / 2;
            double RTrigger1 = -gamepad1.right_trigger / 2;

            boolean dpadUp1 = gamepad1.dpad_up;
            boolean dpadRight1 = gamepad1.dpad_right;
            boolean dpadLeft1 = gamepad1.dpad_left;
            boolean dpadDown1 = gamepad1.dpad_down;

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
            

            boolean dpadUP2 = gamepad2.dpad_up;
            boolean dpadDOWN2 =gamepad2.dpad_down;
            boolean dpadRight2 = gamepad2.dpad_right;
            boolean dpadLeft2 = gamepad2.dpad_left;



            //vuforia output
            if (tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    // step through the list of recognitions and display boundary info
                    for (Recognition recognition : updatedRecognitions) {
                        if (recognition.getLabel().equals("Skystone")) {
                            getLeftPos = recognition.getLeft();
                            getRightPos = recognition.getRight();
                        }
                    }
                }
            }

            telemetry.addData("getLeft pos: ", getLeftPos);
            telemetry.addData("getRight pos: ", getRightPos);


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
                blockgrabBlue.setPosition(.5);
            }
            if (y1) { //up (starting)
                blockgrabBlue.setPosition(1);
            }


            if (a1) {
                if (leftfront.getCurrentPosition() > 0 && rightfront.getCurrentPosition() > 0 &&
                        leftback.getCurrentPosition() > 0 && rightback.getCurrentPosition() > 0) {
                    dir = "forward  ";
                }
                else if (leftfront.getCurrentPosition() < 0 && rightfront.getCurrentPosition() < 0 &&
                        leftback.getCurrentPosition() < 0 && rightback.getCurrentPosition() < 0) {
                    dir = "back  ";
                }
                else if (leftfront.getCurrentPosition() > 0 && rightfront.getCurrentPosition() < 0 &&
                        leftback.getCurrentPosition() > 0 && rightback.getCurrentPosition() < 0) {
                    dir = "clock  ";
                }
                else if (leftfront.getCurrentPosition() < 0 && rightfront.getCurrentPosition() > 0 &&
                        leftback.getCurrentPosition() < 0 && rightback.getCurrentPosition() > 0) {
                    dir = "counter  ";
                }
                else if (leftfront.getCurrentPosition() > 0 && rightfront.getCurrentPosition() < 0 &&
                        leftback.getCurrentPosition() < 0 && rightback.getCurrentPosition() > 0) {
                    dir = "right  ";
                }
                else if (leftfront.getCurrentPosition() < 0 && rightfront.getCurrentPosition() > 0 &&
                        leftback.getCurrentPosition() > 0 && rightback.getCurrentPosition() < 0) {
                    dir = "left  ";
                }

                info.add(dir + leftfront.getCurrentPosition() + " " + rightfront.getCurrentPosition()
                        + " " + leftback.getCurrentPosition() + " " + rightback.getCurrentPosition());

                resetMotors();
            }


            telemetry.addData("left front", leftfront.getCurrentPosition());
            telemetry.addData("right front", rightfront.getCurrentPosition());
            telemetry.addData("left back", leftback.getCurrentPosition());
            telemetry.addData("right back", rightback.getCurrentPosition());
            telemetry.addLine("-------------------------");


            if (!info.isEmpty()) {
                for (String data: info) {
                    telemetry.addLine(data);
                }
            }

            telemetry.update();
        }

        //threadSlide.interrupt();
        //threadArm.interrupt();

    }



    public void SetPower(double LFPower, double RFPower, double LBPower, double RBPower) {
        leftfront.setPower(-LFPower);
        leftback.setPower(-LBPower);
        rightfront.setPower(-RFPower);
        rightback.setPower(-RBPower);
    }


}


