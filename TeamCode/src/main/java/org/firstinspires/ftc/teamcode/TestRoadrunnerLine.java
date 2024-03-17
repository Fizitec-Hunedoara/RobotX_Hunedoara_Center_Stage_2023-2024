package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
@Autonomous
public class TestRoadrunnerLine extends LinearOpMode {

    ChestiiDeAutonom cacat = new ChestiiDeAutonom(this);


    @Override
    public void runOpMode() throws InterruptedException {
        cacat.init(hardwareMap);

        SampleMecanumDrive mecanumDrive = new SampleMecanumDrive(hardwareMap);
        waitForStart();
        while(true)
        {
            if(gamepad1.x)
                break;
        }
        Pose2d startPose = new Pose2d(14.783464, -62.73622, Math.toRadians(90));
        mecanumDrive.setPoseEstimate(startPose);
        mecanumDrive.setMotorPowers(-0.4,0.4,-0.4,0.4);
        while(!cacat.isColorWhite())
        {
            telemetry.addData("Val", cacat.isColorWhite());
            mecanumDrive.update();
        }
        mecanumDrive.setMotorPowers(0,0,0,0);
    }

}
