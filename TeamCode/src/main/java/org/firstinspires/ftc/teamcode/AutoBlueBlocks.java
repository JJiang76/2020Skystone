package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled
@Autonomous (name = "blue blocks auto")
public class AutoBlueBlocks extends LinearOpMode {
    DcMotor rightfront;
    DcMotor leftfront;
    DcMotor leftback;
    DcMotor rightback;
    DcMotor armboi;
    Servo blockgrab;
    Servo grabright;
    Servo grableft;

    final int TICKS_PER_INCH = 90;


    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("init");
        telemetry.update();

        rightfront = hardwareMap.get(DcMotor.class, "rightfront");
        leftfront = hardwareMap.get(DcMotor.class, "leftfront");
        leftback = hardwareMap.get(DcMotor.class, "leftback");
        rightback = hardwareMap.get(DcMotor.class, "rightback");
        blockgrab = hardwareMap.get(Servo.class, "servo2");
        armboi = hardwareMap.get(DcMotor.class, "armboi");

        grabright = hardwareMap.get(Servo.class, "servo0");
        grableft = hardwareMap.get(Servo.class, "servo1");

        rightfront.setDirection(DcMotor.Direction.REVERSE);
        rightback.setDirection(DcMotor.Direction.REVERSE);
        leftback.setDirection(DcMotor.Direction.FORWARD);
        leftfront.setDirection(DcMotor.Direction.FORWARD);
        armboi.setDirection(DcMotor.Direction.REVERSE);

        armboi.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        blockgrab.setPosition(1);
        forward(29, 1); //approaches  a block
        blockgrab.setPosition(.25);
        sleep(500);
        //block grabbed

        back(14,1);
        counter(22, 1);
        forward(50, 1);
        blockgrab.setPosition(1);
        //drop off; head back

        back(58, 1);
        clock(22,1);
        forward(15, 1);
        blockgrab.setPosition(.25);
        sleep(500);
        //second block grabbed

        back(14,.75);
        counter(22, 1);
        forward(58, 1);
        blockgrab.setPosition(1);
        //drop off; head back

        back(64, 1);
        clock(22,1);
        forward(15, 1);
        blockgrab.setPosition(.25);
        sleep(500);
        //third block grabbed

        back(14,.75);
        counter(22, 1);
        forward(64, 1);
        blockgrab.setPosition(1);
        //drop off; head back
/*
        back(72, 1);
        clock(22,1);
        forward(15, 1);
        blockgrab.setPosition(.25);
        sleep(500);
        //fourth block grabbed

        back(14,.75);
        counter(22, 1);
        forward(72, 1);
        blockgrab.setPosition(.5);
        //drop off; head back
*/
    }


    //90 degrees = 22.22 in
    //1 inch lateral = 100 ticks

    public void forward(double dist, double speed) {
        move(speed, dist, dist, dist, dist);
    }
    public void back(double dist, double speed) {
        move(speed, -dist, -dist, -dist, -dist);
    }
    public void counter(double dist, double speed) {
        move(speed, -dist, -dist, dist, dist);
    }
    public void clock(double dist, double speed){ move(speed, dist, dist, -dist, -dist); }
    public void right(double dist, double speed) { move(speed, -dist, dist, dist, -dist); }
    public void left(double dist, double speed) { move(speed, dist, -dist, -dist, dist); }




    public void move(double speed, double distRF, double distRB, double distLF, double distLB) {
        rightfront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightback.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftfront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftback.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int posRF = (int)(distRF * TICKS_PER_INCH);
        int posRB = (int)(distRB * TICKS_PER_INCH);
        int posLF = (int)(distLF * TICKS_PER_INCH);
        int posLB = (int)(distLB * TICKS_PER_INCH);


        rightfront.setTargetPosition(posRF);
        rightback.setTargetPosition(posRB);
        leftfront.setTargetPosition(posLF);
        leftback.setTargetPosition(posLB);

        rightfront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightback.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftfront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftback.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        rightfront.setPower(speed);
        rightback.setPower(speed);
        leftfront.setPower(speed);
        leftback.setPower(speed);



        while(rightfront.isBusy() && rightback.isBusy() && leftfront.isBusy() && leftback.isBusy()) {

        }

        rightfront.setPower(0);
        rightback.setPower(0);
        leftfront.setPower(0);
        leftback.setPower(0);
    }

    public void SetPower(double LFPower, double RFPower, double LBPower, double RBPower) {
        leftfront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftback.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightfront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightback.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        leftfront.setPower(LFPower);
        leftback.setPower(LBPower);
        rightfront.setPower(RFPower);
        rightback.setPower(RBPower);
    }
}
