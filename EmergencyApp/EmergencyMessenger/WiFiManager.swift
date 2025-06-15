import Foundation
import SystemConfiguration.CaptiveNetwork
import CoreLocation
import SwiftUI // For @Published

class WiFiManager: NSObject, ObservableObject, CLLocationManagerDelegate {
    @Published var currentSSID: String?
    @Published var isConnectedToTargetWiFi = false
    @Published var locationAuthorizationStatus: CLAuthorizationStatus = .notDetermined

    private let targetSSID = "Emergency_KU"
    private var locationManager: CLLocationManager?

    override init() {
        super.init()
        locationManager = CLLocationManager()
        locationManager?.delegate = self
        locationManager?.requestWhenInUseAuthorization() // Request permission
    }

    func fetchCurrentSSID() {
        // Getting SSID requires location services to be enabled and authorized.
        guard locationAuthorizationStatus == .authorizedWhenInUse || locationAuthorizationStatus == .authorizedAlways else {
            print("Location services not authorized for SSID fetching.")
            self.currentSSID = nil
            self.isConnectedToTargetWiFi = false
            return
        }

        var ssid: String?
        #if !targetEnvironment(simulator)
        // CNCopyCurrentNetworkInfo is not available on the simulator.
        if let interfaces = CNCopySupportedInterfaces() as? [String] {
            for interface in interfaces {
                if let interfaceInfo = CNCopyCurrentNetworkInfo(interface as CFString) as NSDictionary? {
                    ssid = interfaceInfo[kCNNetworkInfoKeySSID as String] as? String
                    break
                }
            }
        }
        #else
        // For simulator testing, you can uncomment and set a mock SSID
        // print("Running on simulator, SSID check is mocked.")
        // ssid = "Emergency_KU" // Or "OtherWifi" to test different states
        #endif
        
        DispatchQueue.main.async {
            self.currentSSID = ssid
            self.isConnectedToTargetWiFi = (ssid == self.targetSSID)
            print("Current SSID: \(ssid ?? "Not Connected"), Target: \(self.targetSSID), Match: \(self.isConnectedToTargetWiFi)")
        }
    }

    // MARK: - CLLocationManagerDelegate
    func locationManagerDidChangeAuthorization(_ manager: CLLocationManager) {
        DispatchQueue.main.async {
            self.locationAuthorizationStatus = manager.authorizationStatus
            print("Location authorization status changed: \(manager.authorizationStatus.rawValue)")
            if manager.authorizationStatus == .authorizedWhenInUse || manager.authorizationStatus == .authorizedAlways {
                self.fetchCurrentSSID() // Fetch SSID once authorized
            } else {
                self.currentSSID = nil
                self.isConnectedToTargetWiFi = false
            }
        }
    }
    
    // Deprecated delegate method for older iOS versions, forwarding to the new one.
    func locationManager(_ manager: CLLocationManager, didChangeAuthorization status: CLAuthorizationStatus) {
        locationManagerDidChangeAuthorization(manager)
    }
} 