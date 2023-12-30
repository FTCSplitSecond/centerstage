package org.firstinspires.ftc.teamcode.Util

import com.arcrobotics.ftclib.kotlin.extensions.util.clamp
import java.lang.Math.atan
import java.lang.Math.sqrt

data class IKResults(val elbowAngle : Double, val telescopeExtension : Double, val wristAngle : Double) {


}
class InverseKinematics {

   companion object {
      val MAX_PIXEL_LEVEL = 11
      fun calculateArmInverseKinematics (pixelLevel : Int) : IKResults {
         val layerHeight = 10.5 + 3.0 * pixelLevel.clamp(0, MAX_PIXEL_LEVEL)
         val layerHeightZ = sqrt(3.0)*layerHeight/2.0
         val layerHeightX = layerHeight/2.0
         val distanceToBackDrop = 8.0 // temporary
         val distanceFromBackToPivot = 2.0 // back of the robot will come from cad
         val zHeightOfPivot = 3.0 // comes from cad
         val totalX = layerHeightX + distanceToBackDrop + distanceFromBackToPivot
         val totalZ = layerHeightZ - zHeightOfPivot
         val elbowAngle = Math.PI - atan(totalZ/totalX)
         val retractedTelescopeLength = 14.5 // temporary
         val telescopeOffset = 1.0 // temporary from cad
         val offsetLength = 0.0 + telescopeOffset * Math.tan(elbowAngle - Math.PI * 5.0/6.0)
         val telescopeExtension = sqrt(Math.pow(totalX, 2.0) + Math.pow(totalZ, 2.0)) - retractedTelescopeLength + offsetLength
         val wristAngle = Math.PI * 2.0/3.0 - elbowAngle
         return IKResults(elbowAngle, telescopeExtension, wristAngle)
      }
   }








}