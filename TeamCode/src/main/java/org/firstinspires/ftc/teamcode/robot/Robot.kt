package org.firstinspires.ftc.teamcode.robot

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.PoseVelocity2d
import org.firstinspires.ftc.robotcore.external.navigation.Velocity
import org.firstinspires.ftc.teamcode.drivetrain.Drivetrain
import org.firstinspires.ftc.teamcode.helpers.MecanumDrive
import org.firstinspires.ftc.teamcode.helpers.PowerVector

interface Robot {
    val drivetrain: MecanumDrive
    var position: Pose2d
    var velocity: PowerVector // = PoseVelocity2d(Vector2d(0.0, 0.0), 0.0),
    val positionHistory: MutableList<Pose2d> //= LinkedList<Pose2d>(),
    val velocityHistory: MutableList<PoseVelocity2d> //= LinkedList<PoseVelocity2d>(),
    fun update()
    fun initialize()
//    fun driveToPosition(position: Pose2d)
}
