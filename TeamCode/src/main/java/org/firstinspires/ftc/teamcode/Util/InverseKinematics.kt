package org.firstinspires.ftc.teamcode.Util

import android.util.Log
import com.arcrobotics.ftclib.kotlin.extensions.util.clamp
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.teamcode.telescope.subsystems.TelescopeConfig
import java.lang.Math.atan
import java.lang.Math.sqrt

data class IKResults(val elbowAngle : Double, val telescopeExtension : Double, val wristAngle : Double) {


}
class InverseKinematics {

   companion object {
      val MAX_PIXEL_LEVEL = 11
      fun calculateArmInverseKinematics (pixelLevel : Int) : IKResults {
         //TODO: remember to update with new versions of cad latest revision v4
         val layerHeight = 10.5 + 3.0 * pixelLevel.clamp(0, MAX_PIXEL_LEVEL)
         val layerHeightZ = sqrt(3.0)*layerHeight/2.0
         val layerHeightX = layerHeight/2.0
         val distanceToBackDrop = 6.5
         val distanceFromBackToPivot = 2.684 //from cad
         val zHeightOfPivot = 3.6 //from cad
         val totalX = layerHeightX + distanceToBackDrop + distanceFromBackToPivot
         val totalZ = layerHeightZ - zHeightOfPivot
         val elbowAngle = Math.PI - atan(totalZ/totalX)
         val retractedTelescopeLength = 13.5
         val telescopeOffset = 1.25
         val offsetLength = telescopeOffset * Math.tan(elbowAngle - Math.PI * 5.0/6.0)
         val telescopeExtension = sqrt(Math.pow(totalX, 2.0) + Math.pow(totalZ, 2.0)) - retractedTelescopeLength + offsetLength + TelescopeConfig.IK_OFFSET_CONFIG
         val wristAngle = Math.PI * 2.0/3.0 - elbowAngle
         Log.i("IK","pixelLevel = " + pixelLevel)
         Log.i("IK","layerHeight = " + layerHeight)
         Log.i("IK","layerHeightZ = " + layerHeightZ)
         Log.i("IK","layerHeightX = " + layerHeightX)
         Log.i("IK","totalX = " + totalX)
         Log.i("IK","totalZ = " + totalZ)
         Log.i("IK","offsetLength = " + offsetLength)

         val ik = IKResults(AngleUnit.DEGREES.fromRadians(elbowAngle), telescopeExtension, AngleUnit.DEGREES.fromRadians(wristAngle))

         Log.i("IK","elbowAngle = " + ik.elbowAngle)
         Log.i("IK","telescopeExtension = " + ik.telescopeExtension)
         Log.i("IK","wristAngle = " + ik.wristAngle)

         return ik
      }
   }








}