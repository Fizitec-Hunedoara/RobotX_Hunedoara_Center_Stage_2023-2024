package org.firstinspires.ftc.teamcode;

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
public class Autonom_1Stack_Albastru_Outside extends LinearOpMode {
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
                else if (x > 350) {
                    varrez = 3;
                }
                else {
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
        Pose2d startPose = new Pose2d(-38.5118110236, 62.73622, Math.toRadians(270));
        drive.setPoseEstimate(startPose);
        TrajectorySequence ts = drive.trajectorySequenceBuilder(startPose)
                .forward(1)
                .build();
        if(varrez == 3){
            ts = drive.trajectorySequenceBuilder(startPose)
                    .addDisplacementMarker(()->new Thread(() -> {
                        c.setExtensorPower(1,2000);
                    }).start())
                    .lineToLinearHeading(new Pose2d(new Vector2d(-38, 48),Math.toRadians(240)))
                    .build();
        }
        else if(varrez == 2){
            ts = drive.trajectorySequenceBuilder(startPose)
                    .addDisplacementMarker(()->new Thread(() -> {
                        c.setExtensorPower(1,2000);
                    }).start())
                    .lineToLinearHeading(new Pose2d(new Vector2d(-32, 42),Math.toRadians(270)))
                    .build();
        }
        else if(varrez == 1){
            ts = drive.trajectorySequenceBuilder(startPose)
                    .addDisplacementMarker(()->new Thread(() -> {
                        c.setExtensorPower(1,2000);
                    }).start())
                    .lineToLinearHeading(new Pose2d(-41, 32, Math.toRadians(340)))
                    .lineToLinearHeading(new Pose2d(-45, 32, Math.toRadians(340)))
                    .build();
        }
        drive.followTrajectorySequence(ts);
        ts = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .addDisplacementMarker(() -> new Thread(() -> {
                    c.operation_one_pixel();
                    c.kdf(5000);
                    c.protocol_retreat();
                }).start())
                .lineToLinearHeading(new Pose2d(new Vector2d(-30, 58), Math.toRadians(180)))
                .waitSeconds(0.5)
                .splineTo(new Vector2d(-56,44),Math.toRadians(225))
                .setReversed(true)
                .splineTo(new Vector2d(-30,58),Math.toRadians(0))
                .lineToSplineHeading(new Pose2d(new Vector2d(30,58),Math.toRadians(180)))
                .addTemporalMarker(1, 0, () -> new Thread(() -> {
                    c.kdf(1000);
                    c.melctarget(0.87, 700, 3000);
                    c.target(-800, 1300, c.getSlider(), 3000, 10);
                    c.kdf(300);
                    c.target(-300, 1300, c.getSlider(), 3000, 10);
                    c.kdf(500);
                }).start())
                .build();
        drive.followTrajectorySequence(ts);
        if(varrez == 3){
            ts = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .lineToSplineHeading(new Pose2d(new Vector2d(52.5,34),Math.toRadians(180)))
                    .build();
        }
        else if(varrez == 2){
            ts = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .lineToSplineHeading(new Pose2d(new Vector2d(52.5,36),Math.toRadians(180)))
                    .build();
        }
        else if(varrez == 1){
            ts = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .lineToSplineHeading(new Pose2d(new Vector2d(52.5,38),Math.toRadians(180)))
                    .build();
        }
        drive.followTrajectorySequence(ts);
        c.kdf(2000);
        c.deschidereJumate();
        c.kdf(500);
        c.deschidere();
        c.kdf(500);
        c.inchidere();
        c.melctargetRealAngle(430,1000,3000);
        ts = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .setReversed(true)
                .lineToLinearHeading(new Pose2d(new Vector2d(47,60),Math.toRadians(180)))
                .build();
        drive.followTrajectorySequence(ts);
    }
}
