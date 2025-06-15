import SwiftUI
import CoreLocation

struct EmergencyMessage: Codable {
    var name: String
    var location: String
    var latitude: Double
    var longitude: Double
    var emergencyType: Int
    var message: String
}

enum EmergencyType: Int, CaseIterable, Identifiable {
    case medical = 1
    case fire = 2
    case security = 3
    case structural = 4
    case other = 5
    
    var id: Int { self.rawValue }
    
    var description: String {
        switch self {
        case .medical: return "Medical Emergency"
        case .fire: return "Fire Emergency"
        case .security: return "Security Emergency"
        case .structural: return "Structural Damage"
        case .other: return "Other"
        }
    }
    
    var icon: String {
        switch self {
        case .medical: return "cross.fill"
        case .fire: return "flame.fill"
        case .security: return "exclamationmark.shield.fill" 
        case .structural: return "building.fill"
        case .other: return "questionmark.circle.fill"
        }
    }
}

struct TransmissionStatus: Codable {
    var messageId: Int
    var status: String
    var delivered: Bool
}

enum TransmissionState {
    case idle
    case sending
    case waitingForAck
    case delivered
    case failed
}

class LocationManager: NSObject, ObservableObject, CLLocationManagerDelegate {
    private let locationManager = CLLocationManager()
    
    @Published var location: CLLocation?
    @Published var authorizationStatus: CLAuthorizationStatus
    
    override init() {
        authorizationStatus = locationManager.authorizationStatus
        
        super.init()
        locationManager.delegate = self
        locationManager.desiredAccuracy = kCLLocationAccuracyBest
        locationManager.requestWhenInUseAuthorization()
        locationManager.startUpdatingLocation()
    }
    
    func locationManager(_ manager: CLLocationManager, didUpdateLocations locations: [CLLocation]) {
        location = locations.last
    }
    
    func locationManager(_ manager: CLLocationManager, didChangeAuthorization status: CLAuthorizationStatus) {
        authorizationStatus = status
    }
}

class NetworkManager: ObservableObject {
    // The IP address of the Ground Transceiver 1 ESP32 on the Emergency_KU network
    private let serverAddress = "192.168.0.2"
    
    func sendEmergencyMessage(_ message: EmergencyMessage, completion: @escaping (Bool, String, Int?) -> Void) {
        guard let url = URL(string: "http://\(serverAddress)/send") else {
            completion(false, "Invalid server address", nil)
            return
        }
        
        var request = URLRequest(url: url)
        request.httpMethod = "POST"
        request.setValue("application/x-www-form-urlencoded", forHTTPHeaderField: "Content-Type")
        
        // Format the data as form data for the ESP32
        let postData = [
            "name": message.name,
            "location": message.location,
            "latitude": String(message.latitude),
            "longitude": String(message.longitude),
            "emergency": String(message.emergencyType),
            "message": message.message
        ]
        
        let httpBody = postData.map { "\($0.key)=\($0.value)" }.joined(separator: "&")
        request.httpBody = httpBody.data(using: .utf8)
        
        let task = URLSession.shared.dataTask(with: request) { data, response, error in
            if let error = error {
                DispatchQueue.main.async {
                    completion(false, "Network error: \(error.localizedDescription)", nil)
                }
                return
            }
            
            guard let httpResponse = response as? HTTPURLResponse else {
                DispatchQueue.main.async {
                    completion(false, "Invalid server response", nil)
                }
                return
            }
            
            DispatchQueue.main.async {
                if (200...299).contains(httpResponse.statusCode) {
                    // Try to extract messageId from response if available
                    var messageId: Int? = nil
                    if let data = data, let responseString = String(data: data, encoding: .utf8) {
                        if let msgIdStr = responseString.components(separatedBy: ":").last?.trimmingCharacters(in: .whitespacesAndNewlines),
                           let id = Int(msgIdStr) {
                            messageId = id
                        }
                    }
                    completion(true, "Message sent successfully", messageId)
                } else {
                    completion(false, "Server error: \(httpResponse.statusCode)", nil)
                }
            }
        }
        
        task.resume()
    }
    
    func checkMessageStatus(messageId: Int, completion: @escaping (TransmissionStatus?) -> Void) {
        guard let url = URL(string: "http://\(serverAddress)/message-status?id=\(messageId)") else {
            completion(nil)
            return
        }
        
        URLSession.shared.dataTask(with: url) { data, response, error in
            guard let data = data, error == nil,
                  let httpResponse = response as? HTTPURLResponse,
                  (200...299).contains(httpResponse.statusCode) else {
                DispatchQueue.main.async {
                    completion(nil)
                }
                return
            }
            
            do {
                let status = try JSONDecoder().decode(TransmissionStatus.self, from: data)
                DispatchQueue.main.async {
                    completion(status)
                }
            } catch {
                DispatchQueue.main.async {
                    completion(nil)
                }
            }
        }.resume()
    }
}

@main
struct EmergencyMessengerApp: App {
    @Environment(\.scenePhase) var scenePhase
    @StateObject private var wifiManager = WiFiManager()

    var body: some Scene {
        WindowGroup {
            ContentView()
                .environmentObject(wifiManager)
        }
        .onChange(of: scenePhase) { _, newScenePhase in
            if newScenePhase == .active {
                wifiManager.fetchCurrentSSID()
            }
        }
    }
}

struct ContentView: View {
    @StateObject private var locationManager = LocationManager()
    @StateObject private var networkManager = NetworkManager()
    @EnvironmentObject var wifiManager: WiFiManager
    
    @State private var name = ""
    @State private var locationText = ""
    @State private var message = ""
    @State private var selectedEmergencyType = EmergencyType.medical
    
    @State private var transmissionState: TransmissionState = .idle
    @State private var currentMessageId: Int? = nil
    @State private var statusCheckTimer: Timer? = nil
    @State private var transmissionStartTime: Date? = nil
    
    @State private var showAlert = false
    @State private var alertTitle = ""
    @State private var alertMessage = ""
    
    // Computed properties for validation
    private var isFormValid: Bool {
        !name.isEmpty && !locationText.isEmpty && !message.isEmpty
    }
    
    @ViewBuilder
    private var wifiWarningView: some View {
        if !wifiManager.isConnectedToTargetWiFi {
            Section {
                VStack(alignment: .leading, spacing: 8) {
                    HStack {
                        Image(systemName: "wifi.exclamationmark")
                            .foregroundColor(.orange)
                        Text("Incorrect Wi-Fi Network")
                            .font(.headline)
                            .foregroundColor(.orange)
                    }
                    Text("This app requires connection to the 'Emergency_KU' Wi-Fi network to function. Your current network is '\(wifiManager.currentSSID ?? "not connected to Wi-Fi")'.")
                        .font(.callout)
                    
                    if wifiManager.locationAuthorizationStatus != .authorizedWhenInUse && wifiManager.locationAuthorizationStatus != .authorizedAlways {
                        Text("Location permission is needed to check Wi-Fi network. Please grant permission in Settings.")
                            .font(.caption)
                            .foregroundColor(.gray)
                    }

                    Button(action: {
                        if let url = URL(string: UIApplication.openSettingsURLString) {
                            UIApplication.shared.open(url)
                        }
                    }) {
                        Text("Open Settings to Connect Wi-Fi")
                            .frame(maxWidth: .infinity, alignment: .center)
                    }
                    .padding(.top, 5)
                }
            }
            .listRowBackground(Color.orange.opacity(0.1))
        }
    }
    
    @ViewBuilder
    private var yourInformationSection: some View {
        Section(header: Text("Your Information")) {
            TextField("Full Name", text: $name)
                .autocapitalization(.words)
            
            VStack(alignment: .leading) {
                Text("Location Description")
                    .font(.caption)
                    .foregroundColor(.gray)
                
                TextField("Building, floor, room, etc.", text: $locationText)
                    .autocapitalization(.sentences)
            }
        }
    }
    
    @ViewBuilder
    private var emergencyDetailsSection: some View {
        Section(header: Text("Emergency Details")) {
            ScrollView(.horizontal, showsIndicators: false) {
                HStack(spacing: 15) {
                    ForEach(EmergencyType.allCases) { type in
                        VStack {
                            ZStack {
                                Circle()
                                    .fill(selectedEmergencyType == type ? Color.red : Color.gray.opacity(0.2))
                                    .frame(width: 60, height: 60)
                                
                                Image(systemName: type.icon)
                                    .foregroundColor(selectedEmergencyType == type ? .white : .gray)
                                    .font(.system(size: 24))
                            }
                            .onTapGesture {
                                selectedEmergencyType = type
                            }
                            
                            Text(type.description)
                                .font(.caption)
                                .frame(width: 80)
                                .multilineTextAlignment(.center)
                        }
                    }
                }
                .padding(.vertical, 8)
            }
            
            VStack(alignment: .leading) {
                Text("Emergency Message")
                    .font(.caption)
                    .foregroundColor(.gray)
                
                TextEditor(text: $message)
                    .frame(height: 100)
                    .overlay(
                        RoundedRectangle(cornerRadius: 5)
                            .stroke(Color.gray.opacity(0.2), lineWidth: 1)
                    )
            }
        }
    }
    
    @ViewBuilder
    private var gpsLocationSection: some View {
        Section(header: Text("GPS Location")) {
            if locationManager.location != nil {
                HStack {
                    Image(systemName: "location.fill")
                        .foregroundColor(.green)
                    Text("GPS location obtained")
                        .font(.subheadline)
                }
                
                HStack {
                    Text("Latitude:")
                    Spacer()
                    Text("\(locationManager.location?.coordinate.latitude ?? 0, specifier: "%.6f")")
                        .foregroundColor(.gray)
                }
                
                HStack {
                    Text("Longitude:")
                    Spacer()
                    Text("\(locationManager.location?.coordinate.longitude ?? 0, specifier: "%.6f")")
                        .foregroundColor(.gray)
                }
            } else {
                HStack {
                    Image(systemName: "location.slash.fill")
                        .foregroundColor(.red)
                    Text("Waiting for location...")
                        .font(.subheadline)
                }
            }
        }
    }
    
    @ViewBuilder
    private var sendSosButtonSection: some View {
        Section {
            Button(action: sendEmergencyMessage) {
                HStack {
                    Spacer()
                    
                    if transmissionState == .sending || transmissionState == .waitingForAck {
                        ProgressView()
                            .progressViewStyle(CircularProgressViewStyle())
                            .padding(.trailing, 10)
                        Text(transmissionState == .sending ? "SENDING SOS..." : "WAITING FOR CONFIRMATION...")
                            .bold()
                    } else if transmissionState == .delivered {
                        Image(systemName: "checkmark.circle.fill")
                            .foregroundColor(.white)
                            .padding(.trailing, 5)
                        Text("MESSAGE DELIVERED")
                            .bold()
                    } else if transmissionState == .failed {
                        Image(systemName: "exclamationmark.triangle.fill")
                            .foregroundColor(.white)
                            .padding(.trailing, 5)
                        Text("RETRY SEND")
                            .bold()
                    } else {
                        Image(systemName: "arrow.up.circle.fill")
                            .foregroundColor(.white)
                            .padding(.trailing, 5)
                        Text("SEND SOS")
                            .bold()
                    }
                    
                    Spacer()
                }
            }
            .disabled(!canSendMessage || transmissionState == .sending || transmissionState == .waitingForAck)
            .listRowBackground(buttonBackgroundColor(canSend: canSendMessage))
            .foregroundColor(.white)
            
            if transmissionState == .waitingForAck && timeoutReached {
                Button(action: cancelTransmission) {
                    Text("Cancel and try again")
                        .foregroundColor(.red)
                        .frame(maxWidth: .infinity, alignment: .center)
                }
            }
        }
    }
    
    private var canSendMessage: Bool {
        isFormValid && wifiManager.isConnectedToTargetWiFi
    }
    
    private var timeoutReached: Bool {
        guard let startTime = transmissionStartTime else { return false }
        return -startTime.timeIntervalSinceNow > 30 // 30 seconds timeout
    }
    
    var body: some View {
        NavigationView {
            Form {
                wifiWarningView
                yourInformationSection
                emergencyDetailsSection
                gpsLocationSection
                sendSosButtonSection
            }
            .navigationTitle("Emergency SOS")
            .toolbar {
                ToolbarItemGroup(placement: .keyboard) {
                    Spacer()
                    Button("Done") {
                        hideKeyboard()
                    }
                }
            }
            .ignoresSafeArea(.keyboard, edges: .bottom)
            .alert(isPresented: $showAlert) {
                Alert(
                    title: Text(alertTitle),
                    message: Text(alertMessage),
                    dismissButton: .default(Text("OK"))
                )
            }
            .onAppear {
                wifiManager.fetchCurrentSSID()
                if transmissionState != .waitingForAck {
                    transmissionState = .idle
                }
            }
            .onDisappear {
                stopStatusChecking()
            }
        }
    }
    
    private func hideKeyboard() {
        UIApplication.shared.sendAction(#selector(UIResponder.resignFirstResponder), to: nil, from: nil, for: nil)
    }
    
    private var buttonBackgroundColor: Color {
        switch transmissionState {
        case .idle:
            return Color.red
        case .sending, .waitingForAck:
            return Color.orange
        case .delivered:
            return Color.green
        case .failed:
            return Color.red
        }
    }
    
    private func buttonBackgroundColor(canSend: Bool) -> Color {
        if !canSend {
            return Color.gray
        }
        return buttonBackgroundColor
    }
    
    private func sendEmergencyMessage() {
        guard isFormValid else {
            alertTitle = "Missing Information"
            alertMessage = "Please fill in all required fields."
            showAlert = true
            return
        }
        
        // Location is optional from GPS but required from user input
        let coordinates = locationManager.location?.coordinate
        
        transmissionState = .sending
        transmissionStartTime = Date()
        
        let emergencyMessage = EmergencyMessage(
            name: name,
            location: locationText,
            latitude: coordinates?.latitude ?? 0.0,
            longitude: coordinates?.longitude ?? 0.0,
            emergencyType: selectedEmergencyType.rawValue,
            message: message
        )
        
        networkManager.sendEmergencyMessage(emergencyMessage) { success, message, messageId in
            if success, let messageId = messageId {
                self.currentMessageId = messageId
                self.transmissionState = .waitingForAck
                self.startStatusChecking(for: messageId)
                
                // No alert here, UI shows waiting status
            } else {
                self.transmissionState = .failed
                self.alertTitle = "Message Send Failed"
                self.alertMessage = message
                self.showAlert = true
                self.stopStatusChecking()
            }
        }
    }
    
    private func startStatusChecking(for messageId: Int) {
        // Stop any existing timer
        stopStatusChecking()
        
        // Create a new timer that checks status every 2 seconds
        statusCheckTimer = Timer.scheduledTimer(withTimeInterval: 2.0, repeats: true) { _ in
            networkManager.checkMessageStatus(messageId: messageId) { status in
                guard let status = status else {
                    // If we can't check status, keep waiting until timeout
                    if timeoutReached {
                        stopStatusChecking()
                        transmissionState = .failed
                        alertTitle = "Transmission Timeout"
                        alertMessage = "No confirmation was received. You can try sending again."
                        showAlert = true
                    }
                    return
                }
                
                if status.delivered {
                    // Message successfully delivered!
                    transmissionState = .delivered
                    stopStatusChecking()
                    alertTitle = "Success!"
                    alertMessage = "Your emergency message has been successfully delivered to all stations."
                    showAlert = true
                } else if timeoutReached {
                    // Timeout reached, but message still not delivered
                    transmissionState = .failed
                    stopStatusChecking()
                    alertTitle = "Transmission Timeout"
                    alertMessage = "No confirmation was received after 30 seconds. You can try sending again."
                    showAlert = true
                }
            }
        }
    }
    
    private func stopStatusChecking() {
        statusCheckTimer?.invalidate()
        statusCheckTimer = nil
    }
    
    private func cancelTransmission() {
        stopStatusChecking()
        transmissionState = .idle
        currentMessageId = nil
        transmissionStartTime = nil
    }
}

// Preview
struct ContentView_Previews: PreviewProvider {
    static var previews: some View {
        ContentView()
            .environmentObject(WiFiManager())
            .environmentObject(LocationManager())
            .environmentObject(NetworkManager())
    }
} 

