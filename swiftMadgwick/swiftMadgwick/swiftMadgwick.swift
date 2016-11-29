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

//===========================================================================
// ============================== Madgwick IMU ==============================
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
                recipNorm = 0.0
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
                recipNorm = 0.0
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
            recipNorm = 0.0
        }
        
        q0 *= recipNorm
        q1 *= recipNorm
        q2 *= recipNorm
        q3 *= recipNorm
        return Quaternion(_q0: q0, _q1: q1, _q2: q2, _q3: q3)
    }

    
//====================================================================================
// ============================== AHRS algorithm update ==============================
    
    func madgwickAHRS(gx: Double, gy: Double, gz: Double, _ax: Double, _ay: Double, _az: Double, _mx: Double, _my: Double, _mz: Double) -> Quaternion{
        var recipNorm = 0.0
        var ax = _ax, ay = _ay, az = _az
        var mx = _mx, my = _my, mz = _mz
        var s0 = 0.0, s1 = 0.0, s2 = 0.0, s3 = 0.0
        var qDot1 = 0.0, qDot2 = 0.0, qDot3 = 0.0, qDot4 = 0.0;
        var hx = 0.0, hy = 0.0;
        var V_2q0mx = 0.0, V_2q0my = 0.0, V_2q0mz = 0.0, V_2q1mx = 0.0, V_2bx = 0.0, V_2bz = 0.0, V_4bx = 0.0, V_4bz = 0.0, V_2q0 = 0.0, V_2q1 = 0.0, V_2q2 = 0.0, V_2q3 = 0.0, V_2q0q2 = 0.0, V_2q2q3 = 0.0;
        var q0q0 = 0.0, q0q1 = 0.0, q0q2 = 0.0, q0q3 = 0.0, q1q1 = 0.0, q1q2 = 0.0, q1q3 = 0.0, q2q2 = 0.0, q2q3 = 0.0, q3q3 = 0.0;
        
        // Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
        if ((mx == 0.0) && (my == 0.0) && (mz == 0.0)) {
            return self.madgwickAHRS_IMU(gx: gx, gy: gy, gz: gz, ax: ax, ay: ay, az: az)
        }
        
        // Rate of change of quaternion from gyroscope
        qDot1 = 0.5 * (-q1 * gx - q2 * gy - q3 * gz);
        qDot2 = 0.5 * (q0 * gx + q2 * gz - q3 * gy);
        qDot3 = 0.5 * (q0 * gy - q1 * gz + q3 * gx);
        qDot4 = 0.5 * (q0 * gz + q1 * gy - q2 * gx);
        
        // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
        if !(ax == 0.0 && ay == 0.0 && az == 0.0) {
        
            // Normalise accelerometer measurement
            recipNorm = 1.0 / sqrt(ax * ax + ay * ay + az * az)
            ax *= recipNorm
            ay *= recipNorm
            az *= recipNorm
        
            // Normalise magnetometer measurement
            recipNorm = 1.0 / sqrt(mx * mx + my * my + mz * mz)
            mx *= recipNorm;
            my *= recipNorm;
            mz *= recipNorm;
        
            // Auxiliary variables to avoid repeated arithmetic
            V_2q0mx = 2.0 * q0 * mx;
            V_2q0my = 2.0 * q0 * my;
            V_2q0mz = 2.0 * q0 * mz;
            V_2q1mx = 2.0 * q1 * mx;
            V_2q0 = 2.0 * q0;
            V_2q1 = 2.0 * q1;
            V_2q2 = 2.0 * q2;
            V_2q3 = 2.0 * q3;
            V_2q0q2 = 2.0 * q0 * q2;
            V_2q2q3 = 2.0 * q2 * q3;
            q0q0 = q0 * q0;
            q0q1 = q0 * q1;
            q0q2 = q0 * q2;
            q0q3 = q0 * q3;
            q1q1 = q1 * q1;
            q1q2 = q1 * q2;
            q1q3 = q1 * q3;
            q2q2 = q2 * q2;
            q2q3 = q2 * q3;
            q3q3 = q3 * q3;
            
            // Reference direction of Earth's magnetic field
            hx = mx * q0q0 - V_2q0my * q3 + V_2q0mz * q2 + mx * q1q1 + V_2q1 * my * q2 + V_2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
            hy = V_2q0mx * q3 + my * q0q0 - V_2q0mz * q1 + V_2q1mx * q2 - my * q1q1 + my * q2q2 + V_2q2 * mz * q3 - my * q3q3;
            V_2bx = sqrt(hx * hx + hy * hy)
            V_2bz = -V_2q0mx * q2 + V_2q0my * q1 + mz * q0q0 + V_2q1mx * q3 - mz * q1q1 + V_2q2 * my * q3 - mz * q2q2 + mz * q3q3;
            V_4bx = 2.0 * V_2bx;
            V_4bz = 2.0 * V_2bz;
            
            // Gradient decent algorithm corrective step
            s0 = -V_2q2 * (2.0 * q1q3 - V_2q0q2 - ax) + V_2q1 * (2.0 * q0q1 + V_2q2q3 - ay) - V_2bz * q2 * (V_2bx * (0.5 - q2q2 - q3q3) + V_2bz * (q1q3 - q0q2) - mx) + (-V_2bx * q3 + V_2bz * q1) * (V_2bx * (q1q2 - q0q3) + V_2bz * (q0q1 + q2q3) - my) + V_2bx * q2 * (V_2bx * (q0q2 + q1q3) + V_2bz * (0.5 - q1q1 - q2q2) - mz);
            s1 = V_2q3 * (2.0 * q1q3 - V_2q0q2 - ax) + V_2q0 * (2.0 * q0q1 + V_2q2q3 - ay) - 4.0 * q1 * (1 - 2.0 * q1q1 - 2.0 * q2q2 - az) + V_2bz * q3 * (V_2bx * (0.5 - q2q2 - q3q3) + V_2bz * (q1q3 - q0q2) - mx) + (V_2bx * q2 + V_2bz * q0) * (V_2bx * (q1q2 - q0q3) + V_2bz * (q0q1 + q2q3) - my) + (V_2bx * q3 - V_4bz * q1) * (V_2bx * (q0q2 + q1q3) + V_2bz * (0.5 - q1q1 - q2q2) - mz);
            s2 = -V_2q0 * (2.0 * q1q3 - V_2q0q2 - ax) + V_2q3 * (2.0 * q0q1 + V_2q2q3 - ay) - 4.0 * q2 * (1 - 2.0 * q1q1 - 2.0 * q2q2 - az) + (-V_4bx * q2 - V_2bz * q0) * (V_2bx * (0.5 - q2q2 - q3q3) + V_2bz * (q1q3 - q0q2) - mx) + (V_2bx * q1 + V_2bz * q3) * (V_2bx * (q1q2 - q0q3) + V_2bz * (q0q1 + q2q3) - my) + (V_2bx * q0 - V_4bz * q2) * (V_2bx * (q0q2 + q1q3) + V_2bz * (0.5 - q1q1 - q2q2) - mz);
            s3 = V_2q1 * (2.0 * q1q3 - V_2q0q2 - ax) + V_2q2 * (2.0 * q0q1 + V_2q2q3 - ay) + (-V_4bx * q3 + V_2bz * q1) * (V_2bx * (0.5 - q2q2 - q3q3) + V_2bz * (q1q3 - q0q2) - mx) + (-V_2bx * q0 + V_2bz * q2) * (V_2bx * (q1q2 - q0q3) + V_2bz * (q0q1 + q2q3) - my) + V_2bx * q1 * (V_2bx * (q0q2 + q1q3) + V_2bz * (0.5 - q1q1 - q2q2) - mz);
            recipNorm = 1.0 / sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3) // normalise step magnitude
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
        recipNorm = 1.0 / sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3)
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
