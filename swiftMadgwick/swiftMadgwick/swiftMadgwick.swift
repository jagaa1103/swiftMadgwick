//
//  swiftMadgwick.swift
//  swiftMadgwick
//
//  Created by Enkhjargal Gansukh on 29/11/2016.
//  Copyright © 2016 Enkhjargal Gansukh. All rights reserved.
//

import Foundation

class Madgwick {
    
    var sampleFreq = 100.0 // sample frequency Hz
    var betaDef = 1        // 2 * proportional gain
    
    var beta = 1.0     // 2 * proportional gain (Kp)
    
    var q0: Double = 1.0
    var q1: Double = 0.0
    var q2: Double = 0.0
    var q3: Double = 0.0
    
    // IMU ALGORITHM
    func madgwickAHRS_IMU(gx: Double, gy: Double, gz: Double, ax: Double, ay: Double, az: Double) -> Quaternion {
        var _ax = ax, _ay = ay, _az = az
        var recipNorm = 0.0
        var s0 = 0.0, s1 = 0.0, s2 = 0.0, s3 = 0.0
        var qDot1 = 0.0, qDot2 = 0.0, qDot3 = 0.0, qDot4 = 0.0
        var V_2q0 = 0.0, V_2q1 = 0.0, V_2q2 = 0.0, V_2q3 = 0.0, V_4q0 = 0.0, V_4q1 = 0.0, V_4q2 = 0.0, V_8q1 = 0.0, V_8q2 = 0.0
        var q0q0 = 0.0, q1q1 = 0.0, q2q2 = 0.0, q3q3 = 0.0
    
        // Rate of change of quaternion from gyroscope
        qDot1 = 0.5 * (-q1 * gx - q2 * gy - q3 * gz)
        qDot2 = 0.5 * (q0 * gx + q2 * gz - q3 * gy)
        qDot3 = 0.5 * (q0 * gy - q1 * gz + q3 * gx)
        qDot4 = 0.5 * (q0 * gz + q1 * gy - q2 * gx)
    
        // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
        if !(_ax == 0.0 && _ay == 0.0 && _az == 0.0) {
    
            // Normalise accelerometer measurement
            recipNorm = 1.0 / sqrt((_ax * _ax + _ay * _ay + _az * _az))
            if recipNorm.isInfinite {
                recipNorm = 1.0
            }
            _ax *= recipNorm
            _ay *= recipNorm
            _az *= recipNorm
    
            // Auxiliary variables to avoid repeated arithmetic
            V_2q0 = 2.0 * q0
            V_2q1 = 2.0 * q1
            V_2q2 = 2.0 * q2
            V_2q3 = 2.0 * q3
            V_4q0 = 4.0 * q0
            V_4q1 = 4.0 * q1
            V_4q2 = 4.0 * q2
            V_8q1 = 8.0 * q1
            V_8q2 = 8.0 * q2
            q0q0 = q0 * q0
            q1q1 = q1 * q1
            q2q2 = q2 * q2
            q3q3 = q3 * q3
            
            // Gradient decent algorithm corrective step
            s0 = V_4q0 * q2q2 + V_2q2 * _ax + V_4q0 * q1q1 - V_2q1 * _ay
            s1 = V_4q1 * q3q3 - V_2q3 * _ax + 4.0 * q0q0 * q1 - V_2q0 * _ay - V_4q1 + V_8q1 * q1q1 + V_8q1 * q2q2 + V_4q1 * _az
            s2 = 4.0 * q0q0 * q2 + V_2q0 * _ax + V_4q2 * q3q3 - V_2q3 * _ay - V_4q2 + V_8q2 * q1q1 + V_8q2 * q2q2 + V_4q2 * _az
            s3 = 4.0 * q1q1 * q3 - V_2q1 * _ax + 4.0 * q2q2 * q3 - V_2q2 * _ay
            
            
            recipNorm = 1.0 / sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3) // normalise step magnitude
            if recipNorm.isInfinite {
                recipNorm = 1.0
            }
            s0 *= recipNorm
            s1 *= recipNorm
            s2 *= recipNorm
            s3 *= recipNorm
            
            // Apply feedback step
            qDot1 -= beta * s0
            qDot2 -= beta * s1
            qDot3 -= beta * s2
            qDot4 -= beta * s3
        }
    
        // Integrate rate of change of quaternion to yield quaternion
        q0 += qDot1 * (1.0 / sampleFreq)
        q1 += qDot2 * (1.0 / sampleFreq)
        q2 += qDot3 * (1.0 / sampleFreq)
        q3 += qDot4 * (1.0 / sampleFreq)
        
        // Normalise quaternion
        
        recipNorm = 1.0 / sqrt((q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3))
        if recipNorm.isInfinite {
            recipNorm = 1.0
        }
        
        q0 *= recipNorm
        q1 *= recipNorm
        q2 *= recipNorm
        q3 *= recipNorm
        return Quaternion(_q0: q0, _q1: q1, _q2: q2, _q3: q3)
    }
}

struct Quaternion {
    var q0, q1, q2, q3 : Double
    init(_q0: Double, _q1: Double, _q2: Double, _q3: Double ) {
        q0 = _q0
        q1 = _q1
        q2 = _q2
        q3 = _q3
    }
}
