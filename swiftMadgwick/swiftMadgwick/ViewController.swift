//
//  ViewController.swift
//  swiftMadgwick
//
//  Created by Enkhjargal Gansukh on 29/11/2016.
//  Copyright © 2016 Enkhjargal Gansukh. All rights reserved.
//

import UIKit
import CoreMotion

class ViewController: UIViewController {

    @IBOutlet weak var durationTextField: UITextField!
    @IBOutlet weak var startButton: UIButton!
    
    let motionManager = CMMotionManager()
    var sensorDataManager: SensorDataManager? = nil
    
    override func viewDidLoad() {
        super.viewDidLoad()
        // Do any additional setup after loading the view, typically from a nib.
        motionManager.deviceMotionUpdateInterval = 0.01
    }

    override func didReceiveMemoryWarning() {
        super.didReceiveMemoryWarning()
        // Dispose of any resources that can be recreated.
    }
    

    @IBAction func startCalculation(_ sender: Any) {
        if !checksSensors() {
            showAlert(msg: "Sensor is not available to testing. Please check your device is supporting gyro and accel sensors!")
            return
        }
        if self.motionManager.isDeviceMotionActive {
            return
        }
        if durationTextField.text != nil, durationTextField.text != "" {
            sensorDataManager = SensorDataManager()
            if let duration = durationTextField.text {
                Timer.scheduledTimer(withTimeInterval: Double(duration)!, repeats: false, block: { _ in
                    self.stopUpdates()
                    self.sensorDataManager?.printAllData()
                })
                collectMotionData()
            }
        } else {
            showAlert(msg: "Insert duration of motion data set for Madgwick calculation")
        }
    }
    
    func stopUpdates(){
        if self.motionManager.isDeviceMotionActive {
            self.motionManager.stopDeviceMotionUpdates()
        }else if self.motionManager.isGyroActive {
            self.motionManager.stopGyroUpdates()
        }else if self.motionManager.isAccelerometerActive {
            self.motionManager.stopAccelerometerUpdates()
        }
    }
    
    func collectMotionData(){
        motionManager.startDeviceMotionUpdates(to: OperationQueue.main, withHandler: { (data: CMDeviceMotion?, error: Error?) in
            if error != nil {
                self.motionManager.stopDeviceMotionUpdates()
                self.showAlert(msg: error.debugDescription)
            }else{
                if let accelData = data?.gravity, let gyroData = data?.rotationRate, let magnetData = data?.magneticField.field{
                    self.sensorDataManager?.add(ax: accelData.x, ay: accelData.y, az: accelData.z, gx: gyroData.x, gy: gyroData.y, gz: gyroData.z, mx: magnetData.x, my: magnetData.y, mz: magnetData.z)
                }
            }
        })
    }
    
    func checksSensors()->Bool{
        if motionManager.isAccelerometerAvailable,  motionManager.isGyroAvailable {
            return true
        }else{
            return false
        }
    }
    
    func showAlert(msg: String){
        let alert = UIAlertController(title: "Warning", message: msg, preferredStyle: .alert)
        let action = UIAlertAction(title: "Ok", style: .default, handler: nil)
        alert.addAction(action)
        self.present(alert, animated: true, completion: nil)
    }
}

