package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "E auto")
public class e extends LinearOpMode {
    DcMotor yee;
    DcMotor oof;
    @Override
    public void runOpMode() throws InterruptedException {
     yee = hardwareMap.get(DcMotor.class, "Yee");
     oof = hardwareMap.get(DcMotor.class, "Oof");


    }
}
