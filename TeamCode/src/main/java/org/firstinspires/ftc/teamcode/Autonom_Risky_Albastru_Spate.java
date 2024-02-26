package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Var_Blue.CV_detectionType;

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
public class Autonom_Risky_Albastru_Spate extends LinearOpMode {
    double rectx, recty, hperw, x;
    int varrez = 2;
    public OpenCvCamera webcam;
    public PachetelNouAlbastru pipelineAlbastru = new PachetelNouAlbastru(this);
    public ChestiiDeAutonom c = new ChestiiDeAutonom(this);

    @Override
    public void runOpMode() throws InterruptedException {
        c.init(hardwareMap);
        CV_detectionType = Var_Blue.DetectionTypes.NIGHT;
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 2"), cameraMonitorViewId);
        webcam.setPipeline(pipelineAlbastru);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(Var_Blue.Webcam_w, Var_Blue.Webcam_h, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });
        telemetry.addLine("waiting for start:");
        telemetry.update();

        FtcDashboard.getInstance().startCameraStream(webcam, 60);
        while (!isStopRequested() && !isStarted()) {
            try {
                rectx = pipelineAlbastru.getRect().width;
                recty = pipelineAlbastru.getRect().height;
                hperw = recty / rectx;
                telemetry.addData("rectangle width:", rectx);
                telemetry.addData("rectangle height:", recty);
                telemetry.addData("height / width:", hperw);
                x = pipelineAlbastru.getRect().x + pipelineAlbastru.getRect().width / 2.0;
                if (x < 350 && x > 100) {
                    varrez = 2;
                }
                else if (x > 350) {
                    varrez = 3;
                }
                else {
                    varrez = 1;
                }
                telemetry.addData("x:", pipelineAlbastru.getRect().x + pipelineAlbastru.getRect().width / 2);
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
        Pose2d startPose = new Pose2d(-38.5118110236, 62.73622, Math.toRadians(270));
        drive.setPoseEstimate(startPose);
        TrajectorySequence ts = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-39, 40, Math.toRadians(225)))
                .lineToLinearHeading(new Pose2d(-34,40,Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(-34,10,Math.toRadians(180)))
                .addTemporalMarker(1, 0, () -> new Thread(() -> {
                    c.pixel_advance();
                }).start())
                .build();
        if (varrez == 2) {
            ts = drive.trajectorySequenceBuilder(startPose)
                    .lineTo(new Vector2d(-34, 34))
                    .lineTo(new Vector2d(-45, 40))
                    .addDisplacementMarker(() -> new Thread(() -> {
                        c.pixel_advance();
                    }).start())
                    .lineToLinearHeading(new Pose2d(new Vector2d(-45,10),Math.toRadians(180)))
                    .build();
        }
        else if (varrez == 1) {
            ts = drive.trajectorySequenceBuilder(startPose)
                    .lineToLinearHeading(new Pose2d(-38, 40, Math.toRadians(310)))
                    .lineToLinearHeading(new Pose2d(-32, 32, Math.toRadians(340)))
                    .lineTo(new Vector2d(-40, 32))
                    .lineToLinearHeading(new Pose2d(-40, 10, Math.toRadians(170)))
                    .build();
        }
        drive.followTrajectorySequence(ts);
        if(varrez == 1){
            c.pixel_advance();
        }
        ts = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineTo(new Vector2d(-48,10))
                .addTemporalMarker(1, 0, () -> new Thread(() -> {
                    c.kdf(700);
                    c.pixel_retreat();
                }).start())
                .build();
        if (varrez == 2) {
            ts = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .lineToLinearHeading(new Pose2d(-47, 10, Math.toRadians(180)))
                    .addTemporalMarker(1, 0, () -> new Thread(() -> {
                        c.kdf(1000);
                        c.pixel_retreat();
                    }).start())
                    .build();
        }
        else if (varrez == 3) {
            ts = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .lineToLinearHeading(new Pose2d(-47, 10, Math.toRadians(180)))
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
                .lineToLinearHeading(new Pose2d(new Vector2d(20, 6),Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(new Vector2d(53, 40),Math.toRadians(180)))
                .build();
        if (varrez == 2) {
            ts = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .lineToLinearHeading(new Pose2d(new Vector2d(20, 6),Math.toRadians(180)))
                    .lineToLinearHeading(new Pose2d(new Vector2d(53, 36.5),Math.toRadians(180)))
                    .build();
        }
        if (varrez == 3) {
            ts = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .lineToLinearHeading(new Pose2d(new Vector2d(20, 6),Math.toRadians(180)))
                    .lineToLinearHeading(new Pose2d(new Vector2d(53, 32),Math.toRadians(180)))
                    .build();
        }
        drive.followTrajectorySequence(ts);
        c.melctarget(0.85, 3000, 4000);
        c.setMacetaPower(0);
        c.target(-800, 1300, c.getSlider(), 3000, 10);
        c.kdf(200);
        c.target(-300, 1300, c.getSlider(), 5000, 10);
        long lastTime = System.currentTimeMillis();
        while(lastTime + 600 > System.currentTimeMillis()) {
            c.deschidere();
        }
        c.inchidere();
        c.melctarget(2.3, 3000, 3000);
        /*ts = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineTo(new Vector2d(47, 40))
                .lineTo(new Vector2d(47, 10))
                .lineTo(new Vector2d(62, 10))
                .build();
        drive.followTrajectorySequence(ts);*/
    }
}
