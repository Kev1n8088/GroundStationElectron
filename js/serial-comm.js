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
    SENSORS: 156
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

    async connect(portPath = null, baudRate = 57600) {
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
        if (data.length < 53) { // 1 + 52 bytes expected
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
        const timeSinceLaunch = data.readFloatLE(offset);

        const telemetryData = {
            vehicleState,
            quaternion: { w: quatW, x: quatX, y: quatY, z: quatZ },
            acceleration: { x: accelX, y: accelY, z: accelZ },
            velocity: { x: velX, y: velY, z: velZ },
            position: { x: posX, y: posY, z: posZ },
            timeSinceLaunch
        };

        console.log('State Telemetry:', telemetryData);
        this.updateDisplaysWithTelemetry(telemetryData);
    }

    handleSensorData(data) {
        if (data.length < 42) { // 1 + 1 + 40 bytes expected
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
        const gyroBiasRoll = data.readFloatLE(offset);

        const sensorData = {
            failmask,
            sdGood,
            gyro: { yaw: gyroYaw, pitch: gyroPitch, roll: gyroRoll },
            accelerometer: { x: accelX, y: accelY, z: accelZ },
            baroAltitude,
            gyroBias: { yaw: gyroBiasYaw, pitch: gyroBiasPitch, roll: gyroBiasRoll }
        };

        console.log('Sensor Data:', sensorData);
        this.updateDisplaysWithSensors(sensorData);
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

        // // Update 3D scene if it exists
        // if (typeof threeScene !== 'undefined') {
        //     threeScene.updateVehicleTransform();
        // }
        
        // // Update status displays
        // const posXEl = document.getElementById('pos-x');
        // const posYEl = document.getElementById('pos-y');
        // const posZEl = document.getElementById('pos-z');
        
        // if (posXEl) posXEl.textContent = telemetry.position.x.toFixed(1);
        // if (posYEl) posYEl.textContent = telemetry.position.y.toFixed(1);
        // if (posZEl) posZEl.textContent = telemetry.position.z.toFixed(1);

        // // Convert quaternion to euler for display
        // const euler = this.quaternionToEuler(telemetry.quaternion);
        // const rollEl = document.getElementById('roll');
        // const pitchEl = document.getElementById('pitch');
        // const yawEl = document.getElementById('yaw');
        
        // if (rollEl) rollEl.textContent = (euler.roll * 180 / Math.PI).toFixed(1);
        // if (pitchEl) pitchEl.textContent = (euler.pitch * 180 / Math.PI).toFixed(1);
        // if (yawEl) yawEl.textContent = (euler.yaw * 180 / Math.PI).toFixed(1);
        
        // // Update charts if they exist
        // if (typeof updateCharts === 'function') {
        //     updateCharts();
        // }

        threeScene.updateVehicleTransform();
    }

    updateDisplaysWithSensors(sensors) {
        // Update sensor status displays
        // const sensorStatusEl = document.getElementById('sensor-status');
        // if (sensorStatusEl) {
        //     const statusText = sensors.failmask === 0 ? 'ALL OK' : `FAIL: ${sensors.failmask.toString(2)}`;
        //     sensorStatusEl.textContent = statusText;
        //     sensorStatusEl.className = sensors.failmask === 0 ? 'sensor-status ok' : 'sensor-status error';
        // }

        // const sdStatusEl = document.getElementById('sd-status');
        // if (sdStatusEl) {
        //     sdStatusEl.textContent = sensors.sdGood ? 'SD OK' : 'SD FAIL';
        //     sdStatusEl.className = sensors.sdGood ? 'sd-status ok' : 'sd-status error';
        // }

        // console.log('Sensors updated:', {
        //     gyro: sensors.gyro,
        //     accel: sensors.accelerometer,
        //     altitude: sensors.baroAltitude,
        //     sdStatus: sensors.sdGood
        // });
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