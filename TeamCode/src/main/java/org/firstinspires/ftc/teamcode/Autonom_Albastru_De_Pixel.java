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
public class Autonom_Albastru_De_Pixel extends LinearOpMode {
    long lastTime;
    double rectx, recty, hperw,x;
    boolean outOfThread = false;
    int varrez = 2,poz2=100;
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
                    webcam.startStreaming(Var_Red.Webcam_w, Var_Red.Webcam_h, OpenCvCameraRotation.UPRIGHT);
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
                    telemetry.addData("x:", pipelineAlbastru.getRect().x + pipelineAlbastru.getRect().width / 2);
                    if (x == 0) {
                        varrez = 3;
                    }
                    else if (x > 250 && x < 600) {
                        varrez = 2;
                    }
                    else if (x < 250) {
                        varrez = 1;
                    }
                    else {
                        varrez = 3;
                    }
                    telemetry.addData("caz:", varrez);
                } catch (Exception E) {
                    varrez = 1;
                    telemetry.addData("Webcam error:", "please restart");
                    telemetry.update();
                }
                telemetry.update();
            }
            SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
            Pose2d startPose = new Pose2d(14.783464, 62.73622, Math.toRadians(270));
            drive.setPoseEstimate(startPose);
            TrajectorySequence ts = drive.trajectorySequenceBuilder(startPose)
                    .addDisplacementMarker(() -> new Thread(() ->{
                        c.melctarget(0.82, 1300, 3000);
                        c.target(-800, 1300, c.getSlider(), 3000, 10);
                        c.kdf(300);
                        c.target(-300, 1300, c.getSlider(), 3000, 10);
                        c.kdf(500);
                    }).start())
                    .lineToLinearHeading(new Pose2d(new Vector2d(14, 38), Math.toRadians(315)))
                    .build();
            if (varrez == 2) {
                ts = drive.trajectorySequenceBuilder(startPose)
                        .addDisplacementMarker(() -> new Thread(() ->{
                            c.melctarget(0.82, 1300, 3000);
                            c.target(-800, 1300, c.getSlider(), 3000, 10);
                            c.kdf(300);
                            c.target(-300, 1300, c.getSlider(), 3000, 10);
                            c.kdf(500);
                        }).start())
                        .lineTo(new Vector2d(14, 36))
                        .build();
            }
            else if (varrez == 3) {
                ts = drive.trajectorySequenceBuilder(startPose)
                        .lineToLinearHeading(new Pose2d(new Vector2d(14, 48), Math.toRadians(230)))
                        .lineToLinearHeading(new Pose2d(new Vector2d(10, 34), Math.toRadians(200)))
                        .addDisplacementMarker(() -> new Thread(() ->{
                            c.melctarget(0.82, 1300, 3000);
                            c.target(-800, 1300, c.getSlider(), 3000, 10);
                            c.kdf(300);
                            c.target(-300, 1300, c.getSlider(), 3000, 10);
                            c.kdf(500);
                        }).start())
                        .build();
            }
            if (!isStopRequested()) {
                drive.followTrajectorySequence(ts);
            }
            ts = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .lineToLinearHeading(new Pose2d(new Vector2d(52, 28), Math.toRadians(180)))
                    .addTemporalMarker(1, 0, () -> new Thread(() -> {
                        c.setExtensorPower(1,2000);
                    }).start())
                    .build();
            if (varrez == 1) {
                ts = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(new Vector2d(16, 48), Math.toRadians(270)))
                        .lineToLinearHeading(new Pose2d(new Vector2d(52, 40), Math.toRadians(180)))
                        .addTemporalMarker(1, 0, () -> new Thread(() -> {
                            c.setExtensorPower(1,2000);
                        }).start())
                        .build();
            }
            else if (varrez == 2) {
                ts = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(new Vector2d(52, 35), Math.toRadians(180)))
                        .addTemporalMarker(1, 0, () -> new Thread(() -> {
                            c.setExtensorPower(1,2000);
                        }).start())
                        .build();
            }
            if (!isStopRequested()) {
                drive.followTrajectorySequence(ts);
            }
            lastTime = System.currentTimeMillis();
            while(lastTime + 600 > System.currentTimeMillis() && !isStopRequested()) {
                c.deschidere();
            }
            c.inchidere();
            ts = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .addDisplacementMarker(() -> new Thread(() -> {
                        c.melctarget(2.05,1300,3000);
                    }).start())
                    .waitSeconds(0.5)
                    .lineToLinearHeading(new Pose2d(new Vector2d(33,10),Math.toRadians(180)))
                    .addDisplacementMarker(() -> new Thread(() -> {
                        c.operation_pixel();
                    }).start())
                    .lineToLinearHeading(new Pose2d(new Vector2d(-43, 10), Math.toRadians(180)))
                    .build();
            if (!isStopRequested()) {
                drive.followTrajectorySequence(ts);
                c.kdf(700);
            }
            ts = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .lineTo(new Vector2d(30, 11))
                    .addTemporalMarker(1, 0, () -> new Thread(() -> {
                        c.melctarget(0.9, 1300, 3000);
                        c.target(-700, 1300, c.getSlider(), 3000, 10);
                        c.kdf(300);
                        c.target(-200, 1300, c.getSlider(), 3000, 10);
                        c.kdf(500);
                        c.setExtensorPower(-1,2000);
                    }).start())
                    .build();
            if (!isStopRequested()) {
                drive.followTrajectorySequence(ts);
            }
            ts = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .lineTo(new Vector2d(52.5, 34))
                    .build();
            if (varrez == 2) {
                ts = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .lineTo(new Vector2d(52.5, 28))
                        .build();
            }
            if (varrez == 1) {
                ts = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .lineTo(new Vector2d(52.5, 39))
                        .build();
            }
            if (!isStopRequested()) {
                drive.followTrajectorySequence(ts);
            }
            lastTime = System.currentTimeMillis();
            while(lastTime + 500 > System.currentTimeMillis() && !isStopRequested()) {
                c.deschidere();
            }
            c.inchidere();
            c.melctarget(2.3,3000,3000);
    }
    private final Thread Pus_pe_tabla = new Thread(new Runnable() {
        @Override
        public void run() {
            if(!isStopRequested()) {
                outOfThread = false;
                c.melctarget(0.9, 1300, 3000);
                c.target(-800, 1300, c.getSlider(), 3000, 10);
                c.kdf(300);
                c.target(-200, 1300, c.getSlider(), 3000, 10);
                c.kdf(500);
                outOfThread = true;
            }
        }
    });
    private final Thread Brat_jos = new Thread(new Runnable() {
        @Override
        public void run() {
            if(!isStopRequested()) {
                c.kdf(500);
                c.melctarget(2.3, 1300, 3000);
            }
        }
    });
}
