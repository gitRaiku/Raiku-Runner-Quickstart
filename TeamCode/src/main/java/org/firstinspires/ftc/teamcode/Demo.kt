package org.firstinspires.ftc.teamcode

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.canvas.Canvas
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceRunner.COLOR_INACTIVE_TRAJECTORY
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceRunner.COLOR_INACTIVE_TURN
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceRunner.COLOR_INACTIVE_WAIT
import org.firstinspires.ftc.teamcode.trajectorysequence.sequencesegment.TrajectorySegment
import org.firstinspires.ftc.teamcode.trajectorysequence.sequencesegment.TurnSegment
import org.firstinspires.ftc.teamcode.trajectorysequence.sequencesegment.WaitSegment
import org.firstinspires.ftc.teamcode.util.DashboardUtil
import kotlin.math.PI

@TeleOp
class Demo : LinearOpMode() {
    companion object {
        @JvmField
        var P1H: Double = 10.0
        @JvmField
        var P1X: Double = 10.0
        @JvmField
        var P1Y: Double = PI / 4.0

        @JvmField
        var R1X: Double = 10.0
        @JvmField
        var R1Y: Double = PI / 4.0
        @JvmField
        var R2X: Double = 10.0
        @JvmField
        var R2Y: Double = -PI / 4.0

        @JvmField
        var RUN: Boolean = false
    }

    private lateinit var drive: SampleMecanumDrive

    private val ITC = 1 / 2.54
    private fun draw(
            fieldOverlay: Canvas,
            sequence: TrajectorySequence?
    ) {
        if (sequence != null) {
            for (i in 0 until sequence.size()) {
                val segment = sequence[i]
                if (segment is TrajectorySegment) {
                    fieldOverlay.setStrokeWidth(1)
                    fieldOverlay.setStroke(COLOR_INACTIVE_TRAJECTORY)
                    DashboardUtil.drawSampledPath(fieldOverlay, segment.trajectory.path)
                } else if (segment is TurnSegment) {
                    val (x, y) = segment.getStartPose()
                    fieldOverlay.setFill(COLOR_INACTIVE_TURN)
                    fieldOverlay.fillCircle(x * ITC, y * ITC, 2.0)
                } else if (segment is WaitSegment) {
                    val (x, y) = segment.getStartPose()
                    fieldOverlay.setStrokeWidth(1)
                    fieldOverlay.setStroke(COLOR_INACTIVE_WAIT)
                    fieldOverlay.strokeCircle(x * ITC, y * ITC, 3.0)
                }
            }
        }
    }

    override fun runOpMode() {
        drive = SampleMecanumDrive(hardwareMap)

        waitForStart()

        while (!isStopRequested) {
            val vv = SampleMecanumDrive.getVelocityConstraint(10.0, 10.0, 10.0)
            val va = SampleMecanumDrive.getAccelerationConstraint(10.0)
            val vd = SampleMecanumDrive.getAccelerationConstraint(10.0)
            val tb = drive.trajectorySequenceBuilder(Pose2d(0.0, 0.0, 0.0))
                    .funnyRaikuCurveLinear(Pose2d(P1X, P1Y, P1H), Vector2d(R1X, R1Y), Vector2d(R2X, R2Y), vv, va, vd)
                    .build()
            val p = TelemetryPacket()
            val canvas = p.fieldOverlay()
            draw(canvas, tb)
            FtcDashboard.getInstance().sendTelemetryPacket(p)
            if (RUN) {
                RUN = false
                drive.followTrajectorySequence(tb)
            }
        }
    }
}