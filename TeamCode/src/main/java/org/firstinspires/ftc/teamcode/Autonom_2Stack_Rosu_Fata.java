package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Var_Red.CV_detectionType;
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
public class Autonom_2Stack_Rosu_Fata extends LinearOpMode {
    double rectx, recty, hperw,x;
    boolean outOfThread = false;
    int varrez = 2,poz2=100;
    public ChestiiDeAutonom c = new ChestiiDeAutonom(this);
    public OpenCvCamera webcam;
    public PachetelNouRosu pipelineRosu = new PachetelNouRosu(this);
    @Override
    public void runOpMode() throws InterruptedException {
        c.init(hardwareMap);
        CV_detectionType = Var_Red.DetectionTypes.DAY;
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
                if (x < 350 && x > 100) {
                    varrez = 2;
                }
                else if(x > 350){
                    varrez = 3;
                }
                else{
                    varrez = 1;
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
        Pose2d startPose = new Pose2d(14.783464, -62.73622, Math.toRadians(90));
        drive.setPoseEstimate(startPose);
        TrajectorySequence ts = drive.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(0, 0))
                .build();
        if(varrez == 3) {
            ts = drive.trajectorySequenceBuilder(startPose)
                    .lineToLinearHeading(new Pose2d(new Vector2d(14, -38), Math.toRadians(45)))
                    .addTemporalMarker(1, 0, () -> new Thread(() -> {
                        c.kdf(1500);
                        c.melctarget(0.8, 1300, 3000);
                        c.target(-800, 1300, c.getSlider(), 3000, 10);
                        c.kdf(300);
                        c.target(-250, 1300, c.getSlider(), 3000, 10);
                        c.kdf(500);
                    }).start())
                    .build();
        }
        if (varrez == 2) {
            ts = drive.trajectorySequenceBuilder(startPose)
                    .lineTo(new Vector2d(14, -36))
                    .addTemporalMarker(1, 0, () -> new Thread(() -> {
                        c.melctarget(0.8, 1300, 3000);
                        c.target(-800, 1300, c.getSlider(), 3000, 10);
                        c.kdf(300);
                        c.target(-300, 1300, c.getSlider(), 3000, 10);
                        c.kdf(500);
                    }).start())
                    .build();
        }
        else if (varrez == 1) {
            ts = drive.trajectorySequenceBuilder(startPose)
                    .lineToLinearHeading(new Pose2d(new Vector2d(15, -48), Math.toRadians(130)))
                    .lineToLinearHeading(new Pose2d(new Vector2d(12, -38), Math.toRadians(160)))
                    .addTemporalMarker(1, 0, () -> new Thread(() -> {
                        c.melctarget(0.8, 1300, 3000);
                        c.target(-800, 1300, c.getSlider(), 3000, 10);
                        c.kdf(300);
                        c.target(-300, 1300, c.getSlider(), 3000, 10);
                        c.kdf(500);
                    }).start())
                    .build();
        }
        drive.followTrajectorySequence(ts);
        ts = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(new Vector2d(52, -28), Math.toRadians(180)))
                .build();
        if (varrez == 3) {
            ts = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .lineToLinearHeading(new Pose2d(new Vector2d(16, -48), Math.toRadians(90)))
                    .lineToLinearHeading(new Pose2d(new Vector2d(52, -42), Math.toRadians(180)))
                    .build();
        }
        else if (varrez == 2) {
            ts = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .lineToLinearHeading(new Pose2d(new Vector2d(52, -35), Math.toRadians(180)))
                    .build();
        }
        if (!isStopRequested()) {
            drive.followTrajectorySequence(ts);
        }
        long lastTime = System.currentTimeMillis();
        while(lastTime + 600 > System.currentTimeMillis() && !isStopRequested()) {
            c.deschidere();
        }
        c.inchidere();
        ts = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .addDisplacementMarker(() -> new Thread(() -> {
                    c.melctargetRealAngle(440,1000,3000);
                }).start())
                .waitSeconds(1)
                .splineTo(new Vector2d(10,-57.5),Math.toRadians(180))
                .addDisplacementMarker(() -> new Thread(() -> {
                    c.setExtensorPower(1,2000);
                }).start())
                .lineToLinearHeading(new Pose2d(new Vector2d(-30, -57.5), Math.toRadians(180)))
                .addDisplacementMarker(() -> new Thread(() -> {
                    c.operation_pixel();
                }).start())
                .splineTo(new Vector2d(-56,-44),Math.toRadians(140))
                .build();
        if (!isStopRequested()) {
            drive.followTrajectorySequence(ts);
            ts = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .lineToLinearHeading(new Pose2d(new Vector2d(-54,-37),Math.toRadians(140)))
                    .build();
            drive.followTrajectorySequence(ts);
        }
        ts = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .setReversed(true)
                .splineTo(new Vector2d(-30,-58),Math.toRadians(0))
                .lineToSplineHeading(new Pose2d(new Vector2d(10,-58),Math.toRadians(180)))
                .addTemporalMarker(1, 0, () -> new Thread(() -> {
                    c.kdf(1000);
                    c.melctarget(0.85, 700, 3000);
                    c.target(-800, 1300, c.getSlider(), 3000, 10);
                    c.kdf(300);
                    c.target(-300, 1300, c.getSlider(), 3000, 10);
                    c.kdf(500);
                }).start())
                .build();
        if (!isStopRequested()) {
            drive.followTrajectorySequence(ts);
        }
        if(varrez == 2 || varrez == 1){
            ts = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .setReversed(true)
                    .splineTo(new Vector2d(52.5,-41),Math.toRadians(0))
                    .addTemporalMarker(1,0,() -> new Thread(() -> {
                        c.setExtensorPower(-1,2000);
                    }).start())
                    .build();
        }
        else{
            ts = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .setReversed(true)
                    .splineTo(new Vector2d(52.5,-37),Math.toRadians(0))
                    .addTemporalMarker(1,0,() -> new Thread(() -> {
                        c.setExtensorPower(-1,2000);
                    }).start())
                    .build();
        }
        if (!isStopRequested()) {
            drive.followTrajectorySequence(ts);
            c.kdf(2300);
            c.deschidereJumate();
            c.kdf(200);
            c.deschidere();
            c.kdf(500);
            c.inchidere();
            c.melctargetRealAngle(440, 3000, 3000);
            ts = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .setReversed(true)
                    .lineTo(new Vector2d(47,-60))
                    .build();
            drive.followTrajectorySequence(ts);
        }
    }
    private final Thread Pus_pe_tabla = new Thread(new Runnable() {
        @Override
        public void run() {
            if(!isStopRequested()) {
                outOfThread = false;
                c.melctarget(0.8, 1300, 3000);
                c.target(-800, 1300, c.getSlider(), 3000, 10);
                c.kdf(300);
                c.target(-300, 1300, c.getSlider(), 3000, 10);
                c.kdf(500);
                outOfThread = true;
            }
        }
    });
    private final Thread Brat_jos = new Thread(new Runnable() {
        @Override
        public void run() {
            if(!isStopRequested()) {
                c.inchidere();
                c.kdf(500);
                c.melctarget(2.3, 1300, 3000);
            }
        }
    });
}
