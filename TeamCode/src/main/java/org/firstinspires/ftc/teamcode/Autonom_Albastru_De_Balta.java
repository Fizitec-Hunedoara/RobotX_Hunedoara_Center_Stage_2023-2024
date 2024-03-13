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
public class Autonom_Albastru_De_Balta extends LinearOpMode {
    double rectx, recty, hperw,x;
    boolean outOfThread = false;
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
                x = pipelineAlbastru.getRect().x + pipelineAlbastru.getRect().width/2.0;
                telemetry.addData("x:",pipelineAlbastru.getRect().x + pipelineAlbastru.getRect().width/2);
                if(x==0){
                    varrez = 3;
                }
                else if (x > 250 && x < 600) {
                    varrez = 2;
                }
                else if(x < 250){
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
        Pose2d startPose = new Pose2d(14.783464, 62.73622, Math.toRadians(270));
        drive.setPoseEstimate(startPose);
        TrajectorySequence ts = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(new Vector2d(15, 37),Math.toRadians(315)))
                .addTemporalMarker(1, 0, () -> new Thread(() -> {
                    c.kdf(1500);
                    c.melctarget(0.8, 1300, 3000);
                    c.target(-800, 1300, c.getSlider(), 3000, 10);
                    c.kdf(300);
                    c.target(-300, 1300, c.getSlider(), 3000, 10);
                    c.kdf(500);
                }).start())
                .build();
        if(varrez == 2){
            ts = drive.trajectorySequenceBuilder(startPose)
                    .lineTo(new Vector2d(14, 33))
                    .addTemporalMarker(1, 0, () -> new Thread(() -> {
                        c.kdf(1500);
                        c.melctarget(0.8, 1300, 3000);
                        c.target(-800, 1300, c.getSlider(), 3000, 10);
                        c.kdf(300);
                        c.target(-300, 1300, c.getSlider(), 3000, 10);
                        c.kdf(500);
                    }).start())
                    .build();
        }
        else if(varrez == 3){
            ts = drive.trajectorySequenceBuilder(startPose)
                    .lineToLinearHeading(new Pose2d(new Vector2d(15,48),Math.toRadians(230)))
                    .lineToLinearHeading(new Pose2d(new Vector2d(10,34),Math.toRadians(200)))
                    .addTemporalMarker(1, 0, () -> new Thread(() -> {
                        c.kdf(1500);
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
                .lineToLinearHeading(new Pose2d(new Vector2d(53, 28.5),Math.toRadians(180)))
                .build();
        if(varrez == 1){
            ts = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .lineToLinearHeading(new Pose2d(new Vector2d(16,48),Math.toRadians(270)))
                    .lineToLinearHeading(new Pose2d(new Vector2d(52, 40),Math.toRadians(180)))
                    .build();
        }
        else if(varrez == 2){
            ts = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .lineToLinearHeading(new Pose2d(new Vector2d(52,34),Math.toRadians(180)))
                    .build();
        }
        drive.followTrajectorySequence(ts);
        long lastTime = System.currentTimeMillis();
        while(lastTime + 500 < System.currentTimeMillis() && !isStopRequested()) {
            c.deschidere();
        }
        Brat_stop.start();
        if(varrez == 3){
            ts = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .lineTo(new Vector2d(47,29))
                    .build();
            drive.followTrajectorySequence(ts);
        }
        ts = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineTo(new Vector2d(47,60))
                .build();
        drive.followTrajectorySequence(ts);
        ts = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineTo(new Vector2d(62,60))
                .build();
        drive.followTrajectorySequence(ts);
    }
    private final Thread Pus_pe_tabla = new Thread(new Runnable() {
        @Override
        public void run() {
            outOfThread = false;
            c.melctarget(0.8,1300,3000);
            c.target(-800,1300,c.getSlider(),3000,10);
            c.kdf(200);
            c.target(-300,1300,c.getSlider(),3000,10);
            c.kdf(500);
            outOfThread = true;
        }
    });
    private final Thread Brat_jos = new Thread(new Runnable() {
        @Override
        public void run() {
            c.kdf(500);
            c.melctarget(2.3,1300,3000);
        }
    });
    private final Thread Brat_stop = new Thread(new Runnable() {
        @Override
        public void run() {
            c.inchidere();
            c.kdf(500);
            c.melctarget(2.3,1300,3000);
        }
    });
}
