package org.firstinspires.ftc.teamcode.Lower.Drive

import com.pedropathing.geometry.Pose
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.extensions.pedro.PedroComponent.Companion.follower
import dev.nextftc.ftc.ActiveOpMode
import org.firstinspires.ftc.teamcode.FieldConstants
import org.firstinspires.ftc.teamcode.SOTMConstants
import kotlin.math.*

/**
 * Drive Subsystem
 * Provides position, velocity, and aiming data from Pedro pathing.
 *
 * VELOCITY FILTERING:
 * Raw velocity from pose deltas is noisy. We apply a first-order low-pass
 * (exponential moving average) so the virtual goal doesn't jitter.
 *   filtered += alpha * (raw - filtered)
 * Alpha is tunable via SOTMConstants.VELOCITY_ALPHA.
 */
object Drive : Subsystem {

    // ==================== POSE STATE ====================
    var currentX = 0.0
    var currentY = 0.0
    var currentHeading = 0.0  // radians

    private var lastX = 0.0
    private var lastY = 0.0
    private var lastTime = 0L

    // ==================== FILTERED VELOCITY ====================
    // These are the values everything else should read.
    // Raw velocity is computed each loop, then filtered before storing here.
    var velocityX = 0.0
        private set
    var velocityY = 0.0
        private set

    // Robot speed magnitude (cached, avoids redundant sqrt calls)
    var speed = 0.0
        private set

    var poseValid = false

    // ==================== POSE SAVING ====================
    var savedPose: Pose? = null

    // ==================== ALLIANCE ====================
    enum class Alliance { RED, BLUE }
    var alliance = Alliance.BLUE

    val goalX: Double get() = if (alliance == Alliance.RED) FieldConstants.RED_GOAL_X else FieldConstants.BLUE_GOAL_X
    val goalY = FieldConstants.GOAL_Y

    // ==================== SHOOTING ZONES ====================
    private val closeShootingPose = Pose(20.0, 125.0, 0.0)
    private val midShootingPose = Pose(69.0, 72.0, 0.0)
    private val farShootingPose = Pose(80.0, 8.0, 0.0)

    private const val CLOSE_THRESHOLD = 15.0
    private const val MID_THRESHOLD = 20.0

    // Cached zone — computed once per update()
    var cachedZone = ShootingZone.FAR
        private set

    enum class ShootingZone { CLOSE, MID, FAR }

    // ==================== INIT ====================

    override fun initialize() {
        savedPose?.let { pose ->
            follower.pose = pose
            currentX = pose.x
            currentY = pose.y
            currentHeading = pose.heading
        }
        velocityX = 0.0
        velocityY = 0.0
        speed = 0.0
    }

    // ==================== UPDATE ====================

    fun update() {
        val pose = follower.pose
        currentX = pose.x
        currentY = pose.y
        currentHeading = pose.heading
        poseValid = true

        // --- Velocity calculation with low-pass filter ---
        val now = System.currentTimeMillis()
        val dt = (now - lastTime) / 1000.0
        if (dt > 0 && dt < 0.2) {
            val rawVelX = (currentX - lastX) / dt
            val rawVelY = (currentY - lastY) / dt

            // Exponential moving average filter
            val alpha = SOTMConstants.VELOCITY_ALPHA
            velocityX += alpha * (rawVelX - velocityX)
            velocityY += alpha * (rawVelY - velocityY)
        }

        // Cache speed magnitude — used by AutoAim and telemetry
        speed = sqrt(velocityX * velocityX + velocityY * velocityY)

        lastX = currentX
        lastY = currentY
        lastTime = now

        // Cache zone once per loop
        cachedZone = computeShootingZone()
    }

    // ==================== POSE SAVE/LOAD ====================

    fun savePose() {
        savedPose = follower.pose
    }

    fun loadSavedPose() {
        savedPose?.let { pose ->
            follower.pose = pose
            currentX = pose.x
            currentY = pose.y
            currentHeading = pose.heading
        }
    }

    // ==================== DISTANCE TO GOAL ====================

    fun distanceToGoal(): Double {
        val dx = goalX - currentX
        val dy = goalY - currentY
        return sqrt(dx * dx + dy * dy)
    }

    fun distanceToGoalMeters(): Double = distanceToGoal() * 0.0254

    // ==================== VIRTUAL GOAL (SOTM) ====================

    /**
     * Compute time of flight based on distance.
     * Linear model: tof = base + scale * distanceMeters
     */
    fun estimateTimeOfFlight(distanceMeters: Double): Double {
        return SOTMConstants.TOF_BASE + SOTMConstants.TOF_DISTANCE_SCALE * distanceMeters
    }

    /**
     * Get virtual goal X position for SOTM.
     * Subtracts robot velocity * TOF to cancel ball drift.
     * LEAD_GAIN scales the compensation (1.0 = physics-perfect).
     */
    fun getVirtualGoalX(tof: Double): Double {
        return goalX - velocityX * tof * SOTMConstants.LEAD_GAIN
    }

    fun getVirtualGoalY(tof: Double): Double {
        return goalY - velocityY * tof * SOTMConstants.LEAD_GAIN
    }

    /**
     * Distance to virtual goal (for flywheel/hood calculations)
     */
    fun distanceToVirtualGoal(tof: Double): Double {
        val dx = getVirtualGoalX(tof) - currentX
        val dy = getVirtualGoalY(tof) - currentY
        return sqrt(dx * dx + dy * dy)
    }

    /**
     * Field-relative angle to virtual goal (radians)
     * Used by AutoAim to tell the turret where to point.
     */
    fun angleToVirtualGoalRad(tof: Double): Double {
        val dx = getVirtualGoalX(tof) - currentX
        val dy = getVirtualGoalY(tof) - currentY
        return atan2(dy, dx)
    }

    /**
     * Field-relative angle to real goal (radians)
     */
    fun angleToGoalRad(): Double {
        val dx = goalX - currentX
        val dy = goalY - currentY
        return atan2(dy, dx)
    }

    // ==================== SHOOTING ZONES ====================

    private fun distanceTo(pose: Pose): Double {
        val dx = pose.x - currentX
        val dy = pose.y - currentY
        return sqrt(dx * dx + dy * dy)
    }

    private fun computeShootingZone(): ShootingZone {
        val distClose = distanceTo(closeShootingPose)
        val distMid = distanceTo(midShootingPose)
        return when {
            distClose <= CLOSE_THRESHOLD -> ShootingZone.CLOSE
            distMid <= MID_THRESHOLD -> ShootingZone.MID
            else -> ShootingZone.FAR
        }
    }

    fun getShootingZone(): ShootingZone = cachedZone

    fun isInShootingZone(): Boolean {
        return FieldConstants.isInMainShootingZone(currentX, currentY)
    }

    // ==================== PERIODIC ====================

    override fun periodic() {
        update()

        ActiveOpMode.telemetry.run {
            addData("Drive/Valid", if (poseValid) "YES" else "NO")
            if (poseValid) {
                addData("Drive/X", "%.1f".format(currentX))
                addData("Drive/Y", "%.1f".format(currentY))
                addData("Drive/Heading", "%.1f°".format(Math.toDegrees(currentHeading)))
                addData("Drive/Speed", "%.1f in/s".format(speed))
                addData("Drive/Zone", cachedZone.name)
            }
        }
    }
}