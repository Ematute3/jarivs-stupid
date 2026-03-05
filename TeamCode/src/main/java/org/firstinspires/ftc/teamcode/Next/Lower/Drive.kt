package org.firstinspires.ftc.teamcode.Lower.Drive

import com.pedropathing.geometry.Pose
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.extensions.pedro.PedroComponent
import dev.nextftc.extensions.pedro.PedroComponent.Companion.follower
import dev.nextftc.ftc.ActiveOpMode
import org.firstinspires.ftc.teamcode.FieldConstants
import org.firstinspires.ftc.teamcode.SOTMConstants
import kotlin.math.*

/**
 * Drive Subsystem
 * Provides all position and aiming data from Pedro pathing
 * 
 * KEY FEATURES:
 * 1. Pose tracking - tracks robot position on field
 * 2. Pose saving - saves pose between auto and teleop
 * 3. Shooting zones - determines which shooting zone robot is in
 * 4. Distance calculations - distance to goal, virtual goal for SOTM
 * 
 * POSE SAVING:
 * - Call savePose() at end of auto to save current position
 * - Call loadSavedPose() at start of teleop to restore position
 * - This ensures robot knows where it is when switching from auto to teleop
 * 
 * SHOOTING ZONES:
 * - Close zone: around position (20, 125)
 * - Mid zone: around position (69, 72)
 * - Far zone: around position (80, 8)
 * - getShootingZone() returns which zone robot is closest to
 */
object Drive : Subsystem {

    // ==================== STATE ====================
    // Current robot position on field (in inches)
    var currentX = 0.0
    var currentY = 0.0
    var currentHeading = 0.0  // in radians

    // Previous position for velocity calculation
    private var lastX = 0.0
    private var lastY = 0.0
    private var lastTime = 0L

    // Robot velocity (inches per second)
    var velocityX = 0.0
    var velocityY = 0.0

    // Whether pose tracking is valid
    var poseValid = false

    // ==================== POSE SAVING ====================
    // Saved pose for auto-to-teleop transfer
    // Set at end of auto, read at start of teleop
    var savedPose: Pose? = null

    // ==================== ALLIANCE ====================
    // Red or Blue alliance - determines which goal to aim at
    enum class Alliance { RED, BLUE }
    var alliance = Alliance.BLUE

    // Goal position based on alliance
    val goalX: Double get() = if (alliance == Alliance.RED) FieldConstants.RED_GOAL_X else FieldConstants.BLUE_GOAL_X
    val goalY = FieldConstants.GOAL_Y

    // ==================== SHOOTING ZONES ====================
    // Exact positions for each shooting zone (inches)
    // These are where robot shoots best from
    val closeShootingPose = Pose(20.0, 125.0, 0.0)   // Close range shooting
    val midShootingPose = Pose(69.0, 72.0, 0.0)     // Mid range shooting
    val farShootingPose = Pose(80.0, 8.0, 0.0)       // Far range shooting

    // Distance thresholds to determine zone (inches)
    // If within this distance of a zone pose, use that zone
    const val CLOSE_THRESHOLD = 15.0
    const val MID_THRESHOLD = 20.0

    /**
     * Initialize drive subsystem
     * Loads saved pose from previous opmode if available
     * This is called automatically at start of teleop
     */
    override fun initialize() {
        // If we have a saved pose from auto, apply it now
        savedPose?.let { pose ->
            follower.pose = pose
            currentX = pose.x
            currentY = pose.y
            currentHeading = pose.heading
        }
    }

    /**
     * Update drive data
     * Should be called every loop
     * Gets position from Pedro pathing system
     */
    fun update() {
        // Get pose from Pedro
        val pose = follower.pose
        currentX = pose.x
        currentY = pose.y
        currentHeading = pose.heading
        poseValid = true

        // Calculate velocity (inches per second)
        // velocity = distance / time
        val now = System.currentTimeMillis()
        val dt = (now - lastTime) / 1000.0  // convert ms to seconds
        if (dt > 0 && dt < 0.2) {  // only if reasonable time passed
            velocityX = (currentX - lastX) / dt
            velocityY = (currentY - lastY) / dt
        }
        
        // Store for next cycle
        lastX = currentX
        lastY = currentY
        lastTime = now
    }

    /**
     * Save current pose
     * Call this at end of autonomous to save robot position
     * Then loadSavedPose() at start of teleop to restore
     * 
     * Usage: Drive.savePose()
     */
    fun savePose() {
        savedPose = follower.pose
    }

    /**
     * Load saved pose
     * Call this at start of teleop to restore position from auto
     * 
     * Usage: Drive.loadSavedPose()
     */
    fun loadSavedPose() {
        savedPose?.let { pose ->
            follower.pose = pose
            currentX = pose.x
            currentY = pose.y
            currentHeading = pose.heading
        }
    }

    /**
     * Clear saved pose
     * Call this if you want to reset saved pose
     */
    fun clearSavedPose() {
        savedPose = null
    }

    // ==================== DISTANCE CALCULATIONS ====================
    
    /**
     * Get distance to goal (inches)
     * Uses current alliance to determine which goal
     */
    fun distanceToGoal(): Double {
        val dx = goalX - currentX
        val dy = goalY - currentY
        return sqrt(dx * dx + dy * dy)
    }

    /**
     * Get distance to goal in meters
     * Used for flywheel calculations
     */
    fun distanceToGoalMeters(): Double = distanceToGoal() * 0.0254

    /**
     * Distance to red goal specifically
     */
    fun distanceToRedGoal(): Double {
        val dx = FieldConstants.RED_GOAL_X - currentX
        val dy = FieldConstants.GOAL_Y - currentY
        return sqrt(dx * dx + dy * dy)
    }

    /**
     * Distance to blue goal specifically
     */
    fun distanceToBlueGoal(): Double {
        val dx = FieldConstants.BLUE_GOAL_X - currentX
        val dy = FieldConstants.GOAL_Y - currentY
        return sqrt(dx * dx + dy * dy)
    }

    // ==================== SHOOTING ZONE DISTANCES ====================
    
    /**
     * Distance to close shooting pose
     */
    fun distanceToClosePose(): Double {
        return sqrt((closeShootingPose.x - currentX)**2 + (closeShootingPose.y - currentY)**2)
    }
    
    /**
     * Distance to mid shooting pose
     */
    fun distanceToMidPose(): Double {
        return sqrt((midShootingPose.x - currentX)**2 + (midShootingPose.y - currentY)**2)
    }
    
    /**
     * Distance to far shooting pose
     */
    fun distanceToFarPose(): Double {
        return sqrt((farShootingPose.x - currentX)**2 + (farShootingPose.y - currentY)**2)
    }

    /**
     * Get current shooting zone
     * Returns CLOSE, MID, or FAR based on which pose is closest
     * 
     * This determines flywheel velocity and hood position for SOTM
     */
    fun getShootingZone(): ShootingZone {
        val distClose = distanceToClosePose()
        val distMid = distanceToMidPose()
        val distFar = distanceToFarPose()
        
        // Return zone with shortest distance
        return when {
            distClose <= CLOSE_THRESHOLD -> ShootingZone.CLOSE
            distMid <= MID_THRESHOLD -> ShootingZone.MID
            else -> ShootingZone.FAR
        }
    }

    /**
     * Shooting zone enum
     * Used for auto-aim in teleop
     */
    enum class ShootingZone { CLOSE, MID, FAR }

    // ==================== ANGLE CALCULATIONS ====================
    
    /**
     * Get angle from robot to goal (degrees)
     * Field-relative angle
     */
    fun angleToGoal(): Double {
        val dx = goalX - currentX
        val dy = goalY - currentY
        return Math.toDegrees(atan2(dy, dx))
    }

    /**
     * Get angle from robot to goal in robot's reference frame
     * Used for turret aiming
     */
    fun angleToGoalRobotFrame(): Double {
        val fieldAngle = angleToGoal()
        val robotHeadingDeg = Math.toDegrees(currentHeading)
        var turretAngle = fieldAngle - robotHeadingDeg - 90.0  // -90 for turret offset
        return normalizeAngleDegrees(turretAngle)
    }

    // ==================== SHOOTING ZONE CHECKS ====================
    
    /**
     * Check if robot is in main shooting zone
     */
    fun isInShootingZone(): Boolean {
        return FieldConstants.isInMainShootingZone(currentX, currentY)
    }

    /**
     * Check if robot is in any shooting zone
     */
    fun isInAnyShootingZone(): Boolean {
        return FieldConstants.isInMainShootingZone(currentX, currentY) ||
                FieldConstants.isInSecondaryShootingZone(currentX, currentY)
    }

    // ==================== SHOOTING ON THE MOVE (SOTM) ====================
    
    /**
     * Calculate virtual goal position for SOTM
     * Accounts for robot movement while ball is in flight
     * 
     * Formula: virtualGoal = actualGoal + robotVelocity * timeOfFlight
     */
    fun getVirtualGoalX(): Double {
        return goalX + velocityX * SOTMConstants.TIME_OF_FLIGHT
    }

    /**
     * Get virtual Y coordinate
     */
    fun getVirtualGoalY(): Double {
        return goalY + velocityY * SOTMConstants.TIME_OF_FLIGHT
    }

    /**
     * Get angle to virtual goal
     * Use this when shooting while moving
     */
    fun angleToVirtualGoal(): Double {
        val vx = getVirtualGoalX()
        val vy = getVirtualGoalY()
        val dx = vx - currentX
        val dy = vy - currentY
        return Math.toDegrees(atan2(dy, dx))
    }

    /**
     * Get distance to virtual goal
     * Use this for velocity when shooting while moving
     */
    fun distanceToVirtualGoal(): Double {
        val vx = getVirtualGoalX()
        val vy = getVirtualGoalY()
        val dx = vx - currentX
        val dy = vy - currentY
        return sqrt(dx * dx + dy * dy)
    }

    // ==================== UTILITIES ====================
    
    /**
     * Normalize angle to -180 to 180 range
     */
    private fun normalizeAngleDegrees(degrees: Double): Double {
        var angle = degrees % 360
        if (angle > 180) angle -= 360
        if (angle < -180) angle += 360
        return angle
    }

    // ==================== TELEMETRY ====================
    
    /**
     * Periodic update - called every loop
     * Updates telemetry with drive data
     */
    override fun periodic() {
        update()

        ActiveOpMode.telemetry.run {
            addData("=== DRIVE ===", "")
            addData("Drive/Valid", if (poseValid) "YES" else "NO")
            if (poseValid) {
                addData("Drive/X", "%.1f".format(currentX))
                addData("Drive/Y", "%.1f".format(currentY))
                addData("Drive/Heading", "%.1f°".format(Math.toDegrees(currentHeading)))
                addData("Drive/In Zone", if (isInShootingZone()) "YES" else "NO")
                addData("Drive/Dist Goal", "%.1f\"".format(distanceToGoal()))
                addData("Drive/Angle Goal", "%.1f°".format(angleToGoal()))
                addData("Drive/Zone", getShootingZone().name)  // Shows CLOSE/MID/FAR
            }
        }
    }
}
