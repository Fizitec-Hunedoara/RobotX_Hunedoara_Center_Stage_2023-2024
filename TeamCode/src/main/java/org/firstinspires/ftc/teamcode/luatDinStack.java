package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.openftc.easyopencv.OpenCvCamera;
@Disabled
@Autonomous
public class luatDinStack extends LinearOpMode {
    double rectx, recty, hperw,x;
    public DcMotorEx motorBR,motorBL,motorFR,motorFL;
    int varrez = 2;
    public OpenCvCamera webcam;
    public PachetelNouAlbastru pipelineRosu = new PachetelNouAlbastru(this);
    public ChestiiDeAutonom c = new ChestiiDeAutonom(this);
    @Override
    public void runOpMode() throws InterruptedException {
        c.init(hardwareMap);
        waitForStart();
        c.melctargetRealAngleAdaptive(424,10,30000);
    }
}
