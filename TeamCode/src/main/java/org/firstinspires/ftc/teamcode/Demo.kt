package org.firstinspires.ftc.teamcode

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive
import org.firstinspires.ftc.teamcode.util.Encoder

class Demo : LinearOpMode() {
    companion object {
        const val LES = "RF"
        const val RES = "RB"
        const val FES = "LB"
        var LER = true
        var RER = true
        var FER = true
    }

    private lateinit var rf: DcMotorEx
    private lateinit var rb: DcMotorEx
    private lateinit var lf: DcMotorEx
    private lateinit var lb: DcMotorEx

    private lateinit var le: Encoder
    private lateinit var re: Encoder
    private lateinit var fe: Encoder

    private lateinit var drive: SampleMecanumDrive

    fun initm(name: String, reversed: Boolean, useEncoder: Boolean, overclock: Boolean): DcMotorEx {
        val m = hardwareMap.get(DcMotorEx::class.java, name)

        if (overclock) {
            val mconf = m.motorType.clone()
            mconf.achieveableMaxRPMFraction = 1.0
            m.motorType = mconf
        }

        if (useEncoder) {
            m.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
            m.mode = DcMotor.RunMode.RUN_USING_ENCODER
        } else {
            m.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        }
        m.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        m.direction = if (reversed) DcMotorSimple.Direction.REVERSE else DcMotorSimple.Direction.FORWARD

        return m
    }

    fun inite(name: String, reversed: Boolean): Encoder {
        val e = Encoder(hardwareMap.get(DcMotorEx::class.java, name))
        e.direction = if (reversed) Encoder.Direction.REVERSE else Encoder.Direction.FORWARD
        return e
    }

    override fun runOpMode() {
        rf = initm("RF", reversed = true, useEncoder = false, overclock = true)
        rb = initm("RB", reversed = true, useEncoder = false, overclock = true)
        lf = initm("LF", reversed = false, useEncoder = false, overclock = true)
        lb = initm("LB", reversed = false, useEncoder = false, overclock = true)
        le = inite(LES, LER)
        re = inite(RES, RER)
        fe = inite(FES, FER)

        drive = SampleMecanumDrive(hardwareMap)
        val vv = SampleMecanumDrive.getVelocityConstraint(10.0, 10.0, 10.0)
        val va = SampleMecanumDrive.getAccelerationConstraint(10.0)
        val vd = SampleMecanumDrive.getAccelerationConstraint(10.0)
        val tb = drive.trajectoryBuilder(Pose2d(0.0, 0.0, 0.0))
                .lineTo(Vector2d(1.0, 0.0), vv, va, null)



    }
}