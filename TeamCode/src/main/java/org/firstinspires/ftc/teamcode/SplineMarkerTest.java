package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
@Disabled
@Autonomous
public class SplineMarkerTest extends LinearOpMode {
    public ChestiiDeAutonom c = new ChestiiDeAutonom(this);
    @Override
    public void runOpMode() throws InterruptedException {
        c.init(hardwareMap);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));
        drive.setPoseEstimate(startPose);
        TrajectorySequence ts = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .addTemporalMarker(1,() -> new Thread(() -> {
                    while(true);
                }))
                .splineTo(new Vector2d(100,0),Math.toRadians(0))
                .build();
        drive.followTrajectorySequence(ts);
    }
}
