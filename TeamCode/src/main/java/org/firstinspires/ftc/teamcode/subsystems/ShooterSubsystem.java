package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.stealthrobotics.library.StealthSubsystem;
import static org.stealthrobotics.library.opmodes.StealthOpMode.telemetry;

public class ShooterSubsystem extends StealthSubsystem {
    private final DcMotorEx shooterMotor;
    private final ServoEx pusherServo;

    //TODO: Tune servo position constants
    private final double PUSHER_CLOSED_POSITION = 0.0;
    private final double PUSHER_OPEN_POSITION = 1.0;

    public ShooterSubsystem(HardwareMap hardwareMap) {
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooterMotor");
        pusherServo = hardwareMap.get(ServoEx.class, "pusherServo");
    }

    private void setPower(double power) {
        shooterMotor.setPower(power);
    }

    public void setPusher(double pos){
        pusherServo.setPosition(pos);
    }
}
