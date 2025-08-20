// LFS LINK-80 Serial communication functionality
const { SerialPort } = require('serialport');

// LFS LINK-80 Protocol Constants
const PACKET_HEADER = 0xAA;
const CRC_POLYNOMIAL = 0xEDB88320;
const MAX_PACKET_SIZE = 255;
const HEADER_SIZE = 3;
const CHECKSUM_SIZE = 4;

// Message Types
const MESSAGE_TYPES = {
    STATE_TELEMETRY: 155,
    SENSORS: 156,
    GPS: 157,
    LANDER: 158,
    KALMAN: 159
};

let packetBuffer = Buffer.alloc(0);

class LFSSerialComm {
    constructor() {
        this.port = null;
        this.isConnected = false;
        this.availablePorts = [];
    }

    async listPorts() {
        try {
            this.availablePorts = await SerialPort.list();
            console.log('Available ports:', this.availablePorts);
            return this.availablePorts;
        } catch (error) {
            console.error('Error listing ports:', error);
            return [];
        }
    }

    async connect(portPath = null, baudRate = 115200) {
        try {
            // If no port specified, try to find one automatically
            if (!portPath) {
                const ports = await this.listPorts();
                if (ports.length === 0) {
                    throw new Error('No serial ports found');
                }
                
                // Look for common Arduino/UAV port names
                const preferredPort = ports.find(port => 
                    port.friendlyName?.toLowerCase().includes('arduino') ||
                    port.friendlyName?.toLowerCase().includes('ch340') ||
                    port.friendlyName?.toLowerCase().includes('cp210') ||
                    port.manufacturerId?.includes('2341') // Arduino VID
                );
                
                portPath = preferredPort ? preferredPort.path : ports[0].path;
            }

            console.log('Attempting to connect to:', portPath);

            this.port = new SerialPort({ 
                path: portPath, 
                baudRate: baudRate,
                autoOpen: false
            });

            // Setup event handlers
            this.setupEventHandlers();

            // Open the port
            await new Promise((resolve, reject) => {
                this.port.open((error) => {
                    if (error) {
                        reject(error);
                    } else {
                        resolve();
                    }
                });
            });

            this.isConnected = true;
            this.updateConnectionStatus(true, portPath);
            console.log('Serial connection established');

        } catch (error) {
            console.error('Serial connection failed:', error);
            this.updateConnectionStatus(false, null, error.message);
            throw error;
        }
    }

    setupEventHandlers() {
        // Raw binary data received
        this.port.on('data', (data) => {
            this.processIncomingData(data);
        });

        // Port errors
        this.port.on('error', (error) => {
            console.error('Serial port error:', error);
            this.isConnected = false;
            this.updateConnectionStatus(false, null, error.message);
        });

        // Port closed
        this.port.on('close', () => {
            console.log('Serial port closed');
            this.isConnected = false;
            this.updateConnectionStatus(false);
        });

        // Port opened
        this.port.on('open', () => {
            console.log('Serial port opened successfully');
        });
    }

    async disconnect() {
        if (this.port && this.port.isOpen) {
            try {
                await new Promise((resolve, reject) => {
                    this.port.close((error) => {
                        if (error) {
                            reject(error);
                        } else {
                            resolve();
                        }
                    });
                });
                this.isConnected = false;
                this.updateConnectionStatus(false);
                console.log('Serial port disconnected');
            } catch (error) {
                console.error('Error disconnecting:', error);
            }
        }
    }

    processIncomingData(data) {
        // Append to buffer
        packetBuffer = Buffer.concat([packetBuffer, data]);
        
        // Process all complete packets
        while (packetBuffer.length >= HEADER_SIZE + CHECKSUM_SIZE) {
            const packet = this.findAndExtractPacket();
            if (packet) {
                this.handleLFSPacket(packet);
            } else {
                break; // No complete packet found
            }
        }
    }

    findAndExtractPacket() {
        // Look for packet header
        let headerIndex = -1;
        for (let i = 0; i < packetBuffer.length; i++) {
            if (packetBuffer[i] === PACKET_HEADER) {
                headerIndex = i;
                break;
            }
        }

        if (headerIndex === -1) {
            // No header found, clear buffer
            packetBuffer = Buffer.alloc(0);
            return null;
        }

        // Remove any data before the header
        if (headerIndex > 0) {
            packetBuffer = packetBuffer.slice(headerIndex);
        }

        // Check if we have enough data for header
        if (packetBuffer.length < HEADER_SIZE) {
            return null;
        }

        const payloadLength = packetBuffer[1];
        const expectedPacketSize = HEADER_SIZE + payloadLength;

        // Check if we have the complete packet
        if (packetBuffer.length < expectedPacketSize) {
            return null;
        }

        // Extract the packet
        const packet = packetBuffer.slice(0, expectedPacketSize);
        packetBuffer = packetBuffer.slice(expectedPacketSize);

        return packet;
    }

    handleLFSPacket(packet) {
        const unpacked = this.unpackLFSPacket(packet);
        
        if (!unpacked.valid) {
            console.error('Invalid LFS packet:', unpacked.error);
            return;
        }

        console.log('Valid LFS packet received, type:', unpacked.messageType);

        switch (unpacked.messageType) {
            case MESSAGE_TYPES.STATE_TELEMETRY:
                this.handleStateTelemetry(unpacked.data);
                break;
            case MESSAGE_TYPES.SENSORS:
                this.handleSensorData(unpacked.data);
                break;
            case MESSAGE_TYPES.GPS:
                this.handleGPSData(unpacked.data);
                break;
            case MESSAGE_TYPES.LANDER:
                this.handleLanderData(unpacked.data);
                break;
            case MESSAGE_TYPES.KALMAN:
                this.handleKalmanData(unpacked.data);
                break;
            default:
                console.log('Unknown message type:', unpacked.messageType);
        }
    }

    unpackLFSPacket(packet) {
        const result = {
            valid: false,
            messageType: 0,
            data: null,
            error: null
        };

        if (packet.length < HEADER_SIZE + CHECKSUM_SIZE) {
            result.error = 'Packet too short';
            return result;
        }

        if (packet[0] !== PACKET_HEADER) {
            result.error = 'Invalid packet header';
            return result;
        }

        const payloadLength = packet[1];
        const cobsOffset = packet[2];

        if (packet.length !== HEADER_SIZE + payloadLength) {
            result.error = 'Packet size mismatch';
            return result;
        }

        // Note: payloadLength includes the checksum
        if (payloadLength < CHECKSUM_SIZE + 2) {
            result.error = 'Payload too short';
            return result;
        }

        // Extract [payload + checksum] section and apply COBS decoding
        const payloadAndChecksum = Buffer.from(packet.slice(HEADER_SIZE));
        
        if (cobsOffset > 0) {
            this.applyCOBSDecoding(payloadAndChecksum, cobsOffset);
        }

        // Split payload and checksum
        const actualPayloadSize = payloadLength - CHECKSUM_SIZE;
        const payload = payloadAndChecksum.slice(0, actualPayloadSize);
        const receivedCrc = payloadAndChecksum.readUInt32LE(actualPayloadSize);

        // Verify checksum
        const calculatedCrc = this.calculateCRC32(payload);
        if (calculatedCrc !== receivedCrc) {
            result.error = 'CRC mismatch';
            return result;
        }

        if (actualPayloadSize < 2) {
            result.error = 'Payload too short for message type';
            return result;
        }

        result.valid = true;
        result.messageType = payload[1];
        result.data = payload.slice(2); // Skip addressing and message type

        return result;
    }

    applyCOBSDecoding(data, firstCobsOffset) {
        let cobsPos = firstCobsOffset;
        
        while (cobsPos < data.length) {
            const offset = data[cobsPos];
            data[cobsPos] = PACKET_HEADER; // Restore original 0xAA
            
            if (offset === 0) {
                break; // No more encoded bytes
            }
            
            cobsPos += offset;
        }
    }

    calculateCRC32(data) {
        let crc = 0xFFFFFFFF;
        
        for (let i = 0; i < data.length; i++) {
            crc ^= data[i];
            for (let j = 0; j < 8; j++) {
                if (crc & 1) {
                    crc = (crc >>> 1) ^ CRC_POLYNOMIAL;
                } else {
                    crc >>>= 1;
                }
            }
        }
        
        return (~crc) >>> 0; // Ensure unsigned 32-bit
    }

    handleStateTelemetry(data) {
        if (data.length < 65) { // 1 + 52 bytes expected
            console.error('State telemetry data too short');
            return;
        }

        let offset = 0;
        
        const vehicleState = data.readInt8(offset); offset += 1;
        const quatW = data.readFloatLE(offset); offset += 4;
        const quatX = data.readFloatLE(offset); offset += 4;
        const quatY = data.readFloatLE(offset); offset += 4;
        const quatZ = data.readFloatLE(offset); offset += 4;
        const accelX = data.readFloatLE(offset); offset += 4;
        const accelY = data.readFloatLE(offset); offset += 4;
        const accelZ = data.readFloatLE(offset); offset += 4;
        const velX = data.readFloatLE(offset); offset += 4;
        const velY = data.readFloatLE(offset); offset += 4;
        const velZ = data.readFloatLE(offset); offset += 4;
        const posX = data.readFloatLE(offset); offset += 4;
        const posY = data.readFloatLE(offset); offset += 4;
        const posZ = data.readFloatLE(offset); offset += 4;
        const timeSinceLaunch = data.readFloatLE(offset);  offset += 4;
        const vehicleMs = data.readUInt32LE(offset);  offset += 4;
        const downCount = data.readUInt32LE(offset); 

        const telemetryData = {
            vehicleState,
            quaternion: { w: quatW, x: quatX, y: quatY, z: quatZ },
            acceleration: { x: accelX, y: accelY, z: accelZ },
            velocity: { x: velX, y: velY, z: velZ },
            position: { x: posX, y: posY, z: posZ },
            timeSinceLaunch, 
            vehicleMs,
            downCount
        };

        console.log('State Telemetry:', telemetryData);
        this.updateDisplaysWithTelemetry(telemetryData);
    }

    handleSensorData(data) {
        if (data.length < 50) { // 1 + 1 + 40 bytes expected
            console.error('Sensor data too short');
            return;
        }

        let offset = 0;
        
        const failmask = data.readInt8(offset); offset += 1;
        const sdGood = data.readUInt8(offset) !== 0; offset += 1;
        const gyroYaw = data.readFloatLE(offset); offset += 4;
        const gyroPitch = data.readFloatLE(offset); offset += 4;
        const gyroRoll = data.readFloatLE(offset); offset += 4;
        const accelX = data.readFloatLE(offset); offset += 4;
        const accelY = data.readFloatLE(offset); offset += 4;
        const accelZ = data.readFloatLE(offset); offset += 4;
        const baroAltitude = data.readFloatLE(offset); offset += 4;
        const gyroBiasYaw = data.readFloatLE(offset); offset += 4;
        const gyroBiasPitch = data.readFloatLE(offset); offset += 4;
        const gyroBiasRoll = data.readFloatLE(offset); offset += 4;
        const vehicleMs = data.readUInt32LE(offset);  offset += 4;
        const downCount = data.readUInt32LE(offset); 

        const sensorData = {
            failmask,
            sdGood,
            gyro: { yaw: gyroYaw, pitch: gyroPitch, roll: gyroRoll },
            accelerometer: { x: accelX, y: accelY, z: accelZ },
            baroAltitude,
            gyroBias: { yaw: gyroBiasYaw, pitch: gyroBiasPitch, roll: gyroBiasRoll }, 
            vehicleMs,
            downCount
        };

        console.log('Sensor Data:', sensorData);
    }
    
    handleGPSData(data) {
        if (data.length < 80){
            console.error('GPS data too short');
            return;
        }

        let offset = 0;
        const satsInView = data.readUInt8(offset); offset += 1;
        const satsUsed = data.readUInt8(offset); offset += 1;
        const gpsQuality = data.readUInt8(offset); offset += 1;

        const latitude = data.readFloatLE(offset); offset += 4;
        const longitude = data.readFloatLE(offset); offset += 4;
        const altitude = data.readFloatLE(offset); offset += 4;
        const accuracy2D = data.readFloatLE(offset); offset += 4;
        const accuracy3D = data.readFloatLE(offset); offset += 4;
        const PDOP = data.readFloatLE(offset); offset += 4;
        const gpsMs = data.readUInt32LE(offset); offset += 4;
        const lastRTCM = data.readUInt32LE(offset); offset += 4;
        const latHome = data.readFloatLE(offset); offset += 4;
        const lonHome = data.readFloatLE(offset); offset += 4;
        const altHome = data.readFloatLE(offset); offset += 4;
        const downVel = data.readFloatLE(offset); offset += 4;
        const eastVel = data.readFloatLE(offset); offset += 4;
        const northVel = data.readFloatLE(offset); offset += 4;
        const relX = data.readFloatLE(offset); offset += 4;
        const relY = data.readFloatLE(offset); offset += 4;
        const relZ = data.readFloatLE(offset); offset += 4;
        const vehicleMs = data.readUInt32LE(offset); offset += 4;
        const downCount = data.readUInt32LE(offset);

        const gpsData = {
            satsInView,
            satsUsed,
            gpsQuality,
            position: { latitude, longitude, altitude },
            relativePosition: { x: relX, y: relY, z: relZ },
            velocity: { x: -downVel, y: eastVel, z: northVel }, // NED to ENU
            accuracy: { horizontal: accuracy2D, vertical: accuracy3D },
            PDOP,
            gpsMs,
            lastRTCM,
            homePosition: { latitude: latHome, longitude: lonHome, altitude: altHome },
            vehicleMs,
            downCount
        };
    }

    handleLanderData(data){
        if (data.length < 89) {
            console.error('Lander data too short');
            return;
        }

        let offset = 0;
        const YTarget = data.readFloatLE(offset); offset += 4;
        const ZTarget = data.readFloatLE(offset); offset += 4;
        const ignitionAlt = data.readFloatLE(offset); offset += 4;
        const apogeeAlt = data.readFloatLE(offset); offset += 4;
        const yawSetpoint = data.readFloatLE(offset); offset += 4;
        const pitchSetpoint = data.readFloatLE(offset); offset += 4;
        const yawCommand = data.readFloatLE(offset); offset += 4;
        const pitchCommand = data.readFloatLE(offset); offset += 4;
        const rollMixedYaw = data.readFloatLE(offset); offset += 4;
        const rollMixedPitch = data.readFloatLE(offset); offset += 4;
        const rollCommand = data.readFloatLE(offset); offset += 4;
        const YProjected = data.readFloatLE(offset); offset += 4;
        const ZProjected = data.readFloatLE(offset); offset += 4;
        const VBAT = data.readFloatLE(offset); offset += 4;
        const thrust = data.readFloatLE(offset); offset += 4;
        const mass = data.readFloatLE(offset); offset += 4;
        const MMOI = data.readFloatLE(offset); offset += 4;
        const momentArm = data.readFloatLE(offset); offset += 4;
        const pyroStatus = data.readUInt8(offset); offset += 1;
        const vehicleMs = data.readUInt32LE(offset); offset += 4;
        const downCount = data.readUInt32LE(offset);

        const landerData = {
            target: { Y: YTarget, Z: ZTarget },
            ignitionAltitude: ignitionAlt,
            apogeeAltitude: apogeeAlt,
            setpoints: { yaw: yawSetpoint, pitch: pitchSetpoint },
            commands: { yaw: yawCommand, pitch: pitchCommand},
            rollMixed: { yaw: rollMixedYaw, pitch: rollMixedPitch },
            rollCommand,
            projected: { Y: YProjected, Z: ZProjected },
            batteryVoltage: VBAT,
            thrust,
            mass,
            momentOfInertia: MMOI,
            momentArm,
            pyroStatus,
            vehicleMs,
            downCount
        }
    }

    handleKalmanData(data){
        if (data.length < 80){
            console.error('Kalman data too short');
            return;
        }

        let offset = 0;
        const accUncX = data.readFloatLE(offset); offset += 4;
        const accUncY = data.readFloatLE(offset); offset += 4;
        const accUncZ = data.readFloatLE(offset); offset += 4;
        const velUncX = data.readFloatLE(offset); offset += 4;
        const velUncY = data.readFloatLE(offset); offset += 4;
        const velUncZ = data.readFloatLE(offset); offset += 4;
        const posUncX = data.readFloatLE(offset); offset += 4;
        const posUncY = data.readFloatLE(offset); offset += 4;
        const posUncZ = data.readFloatLE(offset); offset += 4;
        const accMeasX = data.readFloatLE(offset); offset += 4;
        const accMeasY = data.readFloatLE(offset); offset += 4;
        const accMeasZ = data.readFloatLE(offset); offset += 4;
        const velMeasX = data.readFloatLE(offset); offset += 4;
        const velMeasY = data.readFloatLE(offset); offset += 4;
        const velMeasZ = data.readFloatLE(offset); offset += 4;
        const posMeasX = data.readFloatLE(offset); offset += 4;
        const posMeasY = data.readFloatLE(offset); offset += 4;
        const posMeasZ = data.readFloatLE(offset); offset += 4;
        const vehicleMs = data.readUInt32LE(offset); offset += 4;
        const downCount = data.readUInt32LE(offset);

        const kalmanData = {
            uncertainty: {
                acceleration: { x: accUncX, y: accUncY, z: accUncZ },
                velocity: { x: velUncX, y: velUncY, z: velUncZ },
                position: { x: posUncX, y: posUncY, z: posUncZ }
            },
            measurements: {
                acceleration: { x: accMeasX, y: accMeasY, z: accMeasZ },
                velocity: { x: velMeasX, y: velMeasY, z: velMeasZ },
                position: { x: posMeasX, y: posMeasY, z: posMeasZ }
            },
            vehicleMs,
            downCount
        };
    }

    updateDisplaysWithTelemetry(telemetry) {
        // Update global vehicle state if it exists
        
        if (typeof vehicleState !== 'undefined') {
            vehicleState.quaternion = telemetry.quaternion;
            console.log("Vehicle state updated with telemetry:", telemetry.quaternion);
            // vehicleState.position = telemetry.position;
            // vehicleState.velocity = telemetry.velocity;
            // vehicleState.acceleration = telemetry.acceleration;
        }

        threeScene.updateVehicleTransform();
    }

    quaternionToEuler(q) {
        // Convert quaternion to euler angles (ZYX order)
        const sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
        const cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
        const roll = Math.atan2(sinr_cosp, cosr_cosp);

        const sinp = 2 * (q.w * q.y - q.z * q.x);
        const pitch = Math.abs(sinp) >= 1 ? Math.sign(sinp) * Math.PI / 2 : Math.asin(sinp);

        const siny_cosp = 2 * (q.w * q.z + q.x * q.y);
        const cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
        const yaw = Math.atan2(siny_cosp, cosy_cosp);

        return { roll, pitch, yaw };
    }

    async sendData(data) {
        if (!this.isConnected || !this.port || !this.port.isOpen) {
            console.warn('Cannot send data: port not connected');
            return false;
        }

        try {
            await new Promise((resolve, reject) => {
                this.port.write(data, (error) => {
                    if (error) {
                        reject(error);
                    } else {
                        resolve();
                    }
                });
            });
            console.log('Data sent:', data.length, 'bytes');
            return true;
        } catch (error) {
            console.error('Error sending data:', error);
            return false;
        }
    }

    updateConnectionStatus(connected, portPath = null, error = null) {
        const statusElement = document.getElementById('serial-status');
        const vehicleStatusElement = document.getElementById('vehicle-status');
        
        if (connected) {
            if (statusElement) {
                statusElement.textContent = `Serial: ${portPath || 'CONNECTED'}`;
                statusElement.className = 'serial-status connected';
            }
            if (vehicleStatusElement) {
                vehicleStatusElement.textContent = 'CONNECTED';
            }
        } else {
            if (statusElement) {
                statusElement.textContent = error ? `Serial: ERROR - ${error}` : 'Serial: DISCONNECTED';
                statusElement.className = 'serial-status';
            }
            if (vehicleStatusElement) {
                vehicleStatusElement.textContent = 'DISCONNECTED';
            }
        }
    }

    // Auto-connect to the first available port
    async autoConnect() {
        const ports = await this.listPorts();
        if (ports.length > 0) {
            try {
                await this.connect(ports[0].path);
                return true;
            } catch (error) {
                console.log('Auto-connect failed, will continue with manual connection');
                return false;
            }
        }
        return false;
    }
}

// Global instance
const serialComm = new LFSSerialComm();

// Global functions for HTML interface
async function connectSerial() {
    if (serialComm.isConnected) {
        await serialComm.disconnect();
        return;
    }
    
    try {
        await serialComm.connect();
    } catch (error) {
        alert('Failed to connect to serial port: ' + error.message);
    }
}

async function sendCommand(command) {
    // TODO: Implement LFS LINK-80 command packets
    console.log('Command requested:', command);
    
    if (!serialComm.isConnected) {
        console.log('Command not sent - simulating for demo:', command);
        // Simulate command for demo purposes
        const vehicleStatusEl = document.getElementById('vehicle-status');
        if (vehicleStatusEl) {
            if (command === 'ARM') {
                vehicleStatusEl.textContent = 'ARMED';
            } else if (command === 'DISARM') {
                vehicleStatusEl.textContent = 'DISARMED';
            }
        }
    }
}

module.exports = { LFSSerialComm, serialComm, connectSerial, sendCommand };