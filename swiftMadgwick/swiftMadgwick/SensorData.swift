//
//  SensorData.swift
//  swiftMadgwick
//
//  Created by Enkhjargal Gansukh on 29/11/2016.
//  Copyright © 2016 Enkhjargal Gansukh. All rights reserved.
//

import Foundation

struct SensorDataManager {
    
    let madgwick = Madgwick()
    
    var accelSet = Array<AccelData>()
    var gyroSet = Array<GyroData>()
    var magSet = Array<MagData>()
    var quatSet = Array<QuatData>()
    
    var sensorDataSet = Array<SensorData>()
    
    mutating func add(ax: Double, ay: Double, az: Double, gx: Double, gy: Double, gz: Double, mx: Double?, my: Double?, mz: Double?) {
        let gyro = GyroData(gx: gx, gy: gy, gz: gz)
        let accel = AccelData(ax: ax, ay: ay, az: az)
        var mag: MagData? = nil
        if let x = mx, let y = my, let z = mz {
            mag = MagData(mx: x, my: y, mz: z)
        }
        let quat = calcQuaternion(accel: accel, gyro: gyro, magnet: mag)
        let dataSet = SensorData(_gyro: gyro, _accel: accel, _mag: mag, _quater: quat)
        
        sensorDataSet.append(dataSet)
    }
    
    func calcQuaternion(accel: AccelData, gyro: GyroData, magnet: MagData?) -> QuatData{
        let quaternion: Quaternion
        if let mag = magnet {
            quaternion = madgwick.madgwickAHRS(gx: gyro.x, gy: gyro.y, gz: gyro.z, _ax: accel.x, _ay: accel.y, _az: accel.z, _mx: mag.x, _my: mag.y, _mz: mag.z)
        }else{
            quaternion = madgwick.madgwickAHRS_IMU(gx: gyro.x, gy: gyro.y, gz: gyro.z, ax: accel.x, ay: accel.y, az: accel.z)
        }
        return QuatData(q0: quaternion.q0, q1: quaternion.q1, q2: quaternion.q2, q3: quaternion.q3)
    }
    
    func printAllData(){
        sensorDataSet.forEach({ data in
            print(
                String(format: "%.4f", data.gx) + "\t" +
                String(format: "%.4f", data.gy) + "\t" +
                String(format: "%.4f", data.gz) + "\t" +
                String(format: "%.4f", data.ax) + "\t" +
                String(format: "%.4f", data.ay) + "\t" +
                String(format: "%.4f", data.az) + "\t" +
                String(format: "%.4f", data.q0) + "\t" +
                String(format: "%.4f", data.q1) + "\t" +
                String(format: "%.4f", data.q2) + "\t" +
                String(format: "%.4f", data.q3)
            )
        })
    }
}

struct AccelData {
    var x, y, z : Double
    init(ax: Double, ay: Double, az: Double) {
        x = ax
        y = ay
        z = az
    }
}

struct GyroData {
    var x, y, z : Double
    init(gx: Double, gy: Double, gz: Double) {
        x = gx
        y = gy
        z = gz
    }
}

struct MagData {
    var x, y, z : Double
    init(mx: Double, my: Double, mz: Double) {
        x = mx
        y = my
        z = mz
    }
}

struct QuatData {
    var q0, q1, q2, q3 : Double
    init(q0: Double, q1: Double, q2: Double, q3: Double) {
        self.q0 = q0
        self.q1 = q1
        self.q2 = q2
        self.q3 = q3
    }
}

struct SensorData {
    var gx, gy, gz : Double
    var ax, ay, az : Double
    var mx, my, mz : Double
    var q0, q1, q2, q3 : Double
    
    init(_gyro: GyroData, _accel: AccelData, _mag: MagData?, _quater: QuatData){
        gx = _gyro.x; gy = _gyro.y; gz = _gyro.z
        ax = _accel.x; ay = _accel.y; az = _accel.z
        if let mg = _mag {
            mx = mg.x; my = mg.y; mz = mg.z
        }else{
            mx = 0; my = 0; mz = 0;
        }
        q0 = _quater.q0; q1 = _quater.q1; q2 = _quater.q2; q3 = _quater.q3
    }
}


