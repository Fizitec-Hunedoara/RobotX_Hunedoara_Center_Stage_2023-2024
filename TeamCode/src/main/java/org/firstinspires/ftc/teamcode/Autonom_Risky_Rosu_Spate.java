package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Var_Blue.CV_detectionType;
import static org.firstinspires.ftc.teamcode.Var_Red.Webcam_h;
import static org.firstinspires.ftc.teamcode.Var_Red.Webcam_w;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous
public class Autonom_Risky_Rosu_Spate extends LinearOpMode {
    double rectx, recty, hperw, x;
    int varrez = 2;
    public OpenCvCamera webcam;
    public PachetelNouRosu pipelineRosu = new PachetelNouRosu(this);
    public ChestiiDeAutonom c = new ChestiiDeAutonom(this);

    @Override
    public void runOpMode() throws InterruptedException {
        c.init(hardwareMap);
        Var_Red.CV_detectionType = Var_Red.DetectionTypes.DAY;
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 2"), cameraMonitorViewId);
        webcam.setPipeline(pipelineRosu);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(Webcam_w, Webcam_h, OpenCvCameraRotation.UPRIGHT);
            }
            @Override
            public void onError(int errorCode) {

            }
        });
        telemetry.addLine("waiting for start:");
        telemetry.update();
        FtcDashboard.getInstance().startCameraStream(webcam, 60);
        c.inchidere();
        while (!isStopRequested() && !isStarted()) {
            try {
                rectx = pipelineRosu.getRect().width;
                recty = pipelineRosu.getRect().height;
                hperw = recty / rectx;
                telemetry.addData("rectangle width:", rectx);
                telemetry.addData("rectangle height:", recty);
                telemetry.addData("height / width:", hperw);
                x = pipelineRosu.getRect().x + pipelineRosu.getRect().width/2.0;
                telemetry.addData("x:",pipelineRosu.getRect().x + pipelineRosu.getRect().width/2);
                if(x==0){
                    varrez = 3;
                }
                else if (x < 500 && x > 150) {
                    varrez = 2;
                }
                else if(x < 150){
                    varrez = 1;
                }
                else{
                    varrez = 3;
                }
                telemetry.addData("caz:", varrez);
            }
            catch (Exception E) {
                varrez = 1;
                telemetry.addData("Webcam error:", "please restart");
                telemetry.update();
            }
            telemetry.update();
        }
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(-38.5118110236, -62.73622, Math.toRadians(90));
        drive.setPoseEstimate(startPose);
        TrajectorySequence ts = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-39,-38, Math.toRadians(135)))
                .lineToLinearHeading(new Pose2d(-34,-38,Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(-34,-10,Math.toRadians(180)))
                .build();
        if (varrez == 2) {
            ts = drive.trajectorySequenceBuilder(startPose)
                    .lineTo(new Vector2d(-34, -34))
                    .lineTo(new Vector2d(-45, -40))
                    .lineToLinearHeading(new Pose2d(new Vector2d(-45,-10),Math.toRadians(180)))
                    .build();
        }
        else if (varrez == 3) {
            ts = drive.trajectorySequenceBuilder(startPose)
                    .lineToLinearHeading(new Pose2d(-38, -40, Math.toRadians(50)))
                    .lineToLinearHeading(new Pose2d(-32, -32, Math.toRadians(20)))
                    .lineTo(new Vector2d(-40, -32))
                    .lineToLinearHeading(new Pose2d(-40, -10, Math.toRadians(190)))
                    .build();
        }
        drive.followTrajectorySequence(ts);
        c.pixel_advance_1();
        ts = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineTo(new Vector2d(-48,-10))
                .addTemporalMarker(1, 0, () -> new Thread(() -> {
                    c.kdf(700);
                    c.pixel_retreat();
                }).start())
                .build();
        if (varrez == 2) {
            ts = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .lineToLinearHeading(new Pose2d(-48, -10, Math.toRadians(180)))
                    .addTemporalMarker(1, 0, () -> new Thread(() -> {
                        c.kdf(1000);
                        c.pixel_retreat();
                    }).start())
                    .build();
        }
        else if (varrez == 1) {
            ts = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .lineToLinearHeading(new Pose2d(-47, -10, Math.toRadians(180)))
                    .addTemporalMarker(1, 0, () -> new Thread(() -> {
                        c.kdf(700);
                        c.pixel_retreat();
                    }).start())
                    .build();
        }
        drive.followTrajectorySequence(ts);
        c.setMacetaPower(1);
        c.kdf(200);
        c.setMacetaPower(-1);
        ts = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(new Vector2d(20, -10),Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(new Vector2d(53, -43),Math.toRadians(180)))
                .build();
        if (varrez == 2) {
            ts = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .lineTo(new Vector2d(20, -6))
                    .lineTo(new Vector2d(53, -36))
                    .build();
        }
        if (varrez == 1) {
            ts = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .lineToLinearHeading(new Pose2d(new Vector2d(20, -6),Math.toRadians(180)))
                    .lineToLinearHeading(new Pose2d(new Vector2d(53, -31),Math.toRadians(180)))
                    .build();
        }
        drive.followTrajectorySequence(ts);
        c.melctarget(0.85, 3000, 4000);
        c.setMacetaPower(0);
        c.target(-800, 1300, c.getSlider(), 3000, 10);
        c.kdf(200);
        c.target(-400, 1300, c.getSlider(), 5000, 10);
        long lastTime = System.currentTimeMillis();
        while(lastTime + 500 > System.currentTimeMillis()) {
            c.deschidere();
        }
        c.inchidere();
        c.melctarget(2.3, 3000, 3000);
        ts = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineTo(new Vector2d(47, -40))
                .lineTo(new Vector2d(47, -10))
                .lineTo(new Vector2d(62, -10))
                .build();
        drive.followTrajectorySequence(ts);
    }
}
