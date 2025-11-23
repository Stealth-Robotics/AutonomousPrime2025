package org.firstinspires.ftc.teamcode.subsystems;

import static org.stealthrobotics.library.opmodes.StealthOpMode.telemetry;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.stealthrobotics.library.StealthSubsystem;

public class EncoderSubsystem extends StealthSubsystem {
    private final AnalogInput encoder;
    private final double offset = -99.2903;

    public EncoderSubsystem(HardwareMap hardwareMap) {
        encoder = hardwareMap.get(AnalogInput.class, "encoder");
    }

    public double getAngle() {
        return AngleUnit.normalizeDegrees((encoder.getVoltage()-0.043)/3.1*360 + offset);
    }

    @Override
    public void periodic() {
        telemetry.addData("angle", getAngle());

        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();

        dashboardTelemetry.addData("angle", getAngle());
        dashboardTelemetry.update();
    }
}
