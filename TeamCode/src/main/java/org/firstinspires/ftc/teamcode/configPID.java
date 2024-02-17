package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

@Config
public class configPID {
    public static double kp = 10, ki = 0, kd = 0, kf = 0, vel = 50, p = 0.05, i = 0.001, d = 1, f = 0.75, tolerance = 1, targetPoz = 470;
    public static double pcam = 0, icam = 0, dcam = 0;
}
