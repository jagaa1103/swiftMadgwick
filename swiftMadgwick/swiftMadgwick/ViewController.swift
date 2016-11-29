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
        if self.motionManager.isDeviceMotionActive {
            return
        }
        if durationTextField.text != nil, durationTextField.text != "" {
            if let duration = durationTextField.text {
                Timer.scheduledTimer(withTimeInterval: Double(duration)!, repeats: false, block: { _ in
                    if self.motionManager.isDeviceMotionActive {
                        self.motionManager.stopDeviceMotionUpdates()
                    }else if self.motionManager.isGyroActive {
                        self.motionManager.stopGyroUpdates()
                    }else if self.motionManager.isAccelerometerActive {
                        self.motionManager.stopAccelerometerUpdates()
                    }
                })
                collectMotionData()
            }
        } else {
            showAlert(msg: "Insert duration of motion data set for Madgwick calculation")
        }
    }
    
    
    func collectMotionData(){
        motionManager.startDeviceMotionUpdates(to: OperationQueue.main, withHandler: { (data: CMDeviceMotion?, error: Error?) in
            if error != nil {
                self.motionManager.stopDeviceMotionUpdates()
                self.showAlert(msg: error.debugDescription)
            }else{
                if let accelData = data?.gravity, let gyroData = data?.rotationRate {
                    print("accel: \(accelData.x), \(accelData.y), \(accelData.z), gyro: \(gyroData.x), \(gyroData.y), \(gyroData.z)")
                }
            }
        })
    }
    
    func checksGyro()->Bool{
        if motionManager.isGyroAvailable {
            return true
        }else{
            return false
        }
    }
    
    func gyroDataCollect(){
        motionManager.gyroUpdateInterval = 0.01
        motionManager.startGyroUpdates(to: OperationQueue.main, withHandler: { (data: CMGyroData?, error: Error?) in
            if error != nil {
                self.motionManager.stopGyroUpdates()
                self.showAlert(msg: error.debugDescription)
            }else{
                
                print("\(data?.rotationRate.x), \(data?.rotationRate.y), \(data?.rotationRate.z)")
            }
        })
    }
    
    func accelDataCollect(){
        motionManager.accelerometerUpdateInterval = 0.01
        motionManager.startAccelerometerUpdates(to: OperationQueue.main, withHandler: { (data: CMAccelerometerData?, error: Error?) in
            if error != nil {
                self.motionManager.stopAccelerometerUpdates()
                self.showAlert(msg: error.debugDescription)
            } else {
                print("\(data?.acceleration.x), \(data?.acceleration.y), \(data?.acceleration.z)")
            }
        })
    }
    
    func showAlert(msg: String){
        let alert = UIAlertController(title: "Warning", message: msg, preferredStyle: .alert)
        let action = UIAlertAction(title: "Ok", style: .default, handler: nil)
        alert.addAction(action)
        self.present(alert, animated: true, completion: nil)
    }
}

