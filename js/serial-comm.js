// LFS LINK-80 Serial communication functionality
const { SerialPort } = require('serialport');

function getControlPlots() {
    if (typeof window !== 'undefined' && window.controlPlots) {
        return window.controlPlots;
    }
    // Return fallback object with no-op functions if not available
    return {
        updateAttitudeSetpoints: () => {},
        updateTargetPosition: () => {}, 
        updateCurrentPosition: () => {},
        updateCurrentQuaternion: () => {}
    };
}
// LFS LINK-80 Protocol Constants
const PACKET_HEADER = 0xAA;
const CRC_POLYNOMIAL = 0xEDB88320;
const MAX_PACKET_SIZE = 255;
const HEADER_SIZE = 3;
const CHECKSUM_SIZE = 4;

// Message Types
const MESSAGE_TYPES = {
    // RTCM fragment messages
    RTCM_FRAGMENT_1: 51,
    RTCM_FRAGMENT_2: 52,
    RTCM_FRAGMENT_3: 53,
    RTCM_FRAGMENT_4: 54,
    RTCM_FINAL: 55,
    
    // Telemetry messages
    STATE_TELEMETRY: 155,
    SENSORS: 156,
    GPS: 157,
    LANDER: 158,
    KALMAN: 159
};

// Command Types for uplink
const COMMAND_TYPES = {
    PING: 10,
    ARM: 11,
    DISARM: 12,
    WHEEL_TEST: 20,
    STAB_TEST: 21,
    POS_TEST: 22,
    TRAJ_TEST: 23,
    MISALIGN_TEST: 24,
    TEST_PREP: 25,
    PYRO_TEST: 26  // Added pyro test command
};

// Vehicle state enum mapping
const VEHICLE_STATES = {
    0: 'DISARMED',
    1: 'ARMED', 
    2: 'ALIGN CALC',
    3: 'ASC GUIDE',
    4: 'ASC STAB',
    5: 'DES COAST',
    6: 'DES GUIDE',
    7: 'LANDED',
    64: 'WHEEL TEST',
    65: 'STAB TEST',
    66: 'POS TEST',
    67: 'TRAJ TEST',
    68: 'ALIGN TEST',
    69: 'TEST PREP',
    70: 'PYRO TEST'
};

// Sensor failmask bits
const SENSOR_FAIL_BITS = {
    BAROMETER: 0x01,
    SCH1: 0x02,
    ICM: 0x04,
    MAGNETOMETER: 0x08,
    GPS: 0x10
};

let packetBuffer = Buffer.alloc(0);

let lastTelemetryTime = 0;
let telemetryUpdateInterval = null;

// Add this function to update telemetry age
function updateTelemetryAge() {
    const telemetryAgeElement = document.getElementById('telemetry-age');
    if (!telemetryAgeElement) return;
    
    if (lastTelemetryTime === 0) {
        telemetryAgeElement.textContent = '--';
        telemetryAgeElement.className = 'status-value bad';
        return;
    }
    
    const ageMs = Date.now() - lastTelemetryTime;
    
    // Don't display values above 10 seconds
    if (ageMs > 10000) {
        telemetryAgeElement.textContent = '--';
        telemetryAgeElement.className = 'status-value bad';
        return;
    }
    
    telemetryAgeElement.textContent = `${ageMs}ms`;
    
    // Color coding
    telemetryAgeElement.className = 'status-value';
    if (ageMs <= 100) {
        telemetryAgeElement.classList.add('good');
    } else if (ageMs <= 1000) {
        telemetryAgeElement.classList.add('warning');
    } else {
        telemetryAgeElement.classList.add('bad');
    }
}

class LFSSerialComm {
    constructor() {
        this.port = null;
        this.isConnected = false;
        this.availablePorts = [];
        
        // Store latest telemetry values for cross-referencing between message types
        this.lastAltitude = 0;
        this.lastTimeSinceLaunch = 0;
        this.lastVehicleStateValue = 0;
        this.lastThrust = 0;
        
        // Command packet creation
        this.commandMessageId = 0;
        this.payloadBuffer = Buffer.alloc(255);
        this.packetBuffer = Buffer.alloc(255);
        this.payloadAndChecksumBuffer = Buffer.alloc(255);
        
        // RTCM functionality
        this.rtcmIdCounter = 0;
        this.rtcmEnabled = false;
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
        lastTelemetryTime = Date.now();

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
        if (data.length < 65) {
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
        const timeSinceLaunch = data.readFloatLE(offset); offset += 4;
        const vehicleMs = data.readUInt32LE(offset); offset += 4;
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

        // Store latest values for cross-referencing
        this.lastAltitude = posY; // Y is altitude
        this.lastTimeSinceLaunch = timeSinceLaunch;
        this.lastVehicleStateValue = vehicleState;

        //console.log('State Telemetry:', telemetryData);
        this.updateDisplaysWithTelemetry(telemetryData);
        this.updateVehicleState(vehicleState);
        this.updateLaunchTime(timeSinceLaunch);
        getControlPlots().updateCurrentPosition(telemetryData.position);
        getControlPlots().updateCurrentQuaternion(telemetryData.quaternion);
        
        // Update flight data chart with altitude and time since launch
        if (typeof updateFlightChart !== 'undefined') {
            updateFlightChart(this.lastThrust, this.lastAltitude, this.lastTimeSinceLaunch, this.lastVehicleStateValue);
        }
        
        // Update velocity indicator with vertical velocity (Z component)
        if (typeof velocityIndicator !== 'undefined') {
            velocityIndicator.updateVelocity(telemetryData.velocity.z);
        }
    }

    handleSensorData(data) {
        if (data.length < 50) {
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

        //console.log('Sensor Data:', sensorData);
        this.updateSensorStatus(failmask);
        this.updateSensorPanelData(sensorData);
        
        // Update roll rate indicator with gyro roll data
        if (typeof rollRateIndicator !== 'undefined') {
            rollRateIndicator.updateRollRate(gyroRoll);
        }
    }
    
    handleGPSData(data) {
        if (data.length < 79) {
            console.error('GPS data too short');
            return;
        }

        let offset = 0;
        const satsInView = data.readUInt8(offset); offset += 1;
        const satsUsed = data.readUInt8(offset); offset += 1;
        const gpsQuality = data.readUInt8(offset); offset += 1;
        const currentLat = data.readFloatLE(offset); offset += 4;
        const currentLon = data.readFloatLE(offset); offset += 4;
        const currentAlt = data.readFloatLE(offset); offset += 4;
        const accuracy2D = data.readFloatLE(offset); offset += 4;
        const accuracy3D = data.readFloatLE(offset); offset += 4;
        const pdop = data.readFloatLE(offset); offset += 4;
        const gps_ms = data.readUInt32LE(offset); offset += 4;
        const lastRTCM = data.readUInt32LE(offset); offset += 4;
        const homeLat = data.readFloatLE(offset); offset += 4;
        const homeLon = data.readFloatLE(offset); offset += 4;
        const homeAlt = data.readFloatLE(offset); offset += 4;
        const downVel = data.readFloatLE(offset); offset += 4;
        const eastVel = data.readFloatLE(offset); offset += 4;
        const northVel = data.readFloatLE(offset); offset += 4;
        const relX = data.readFloatLE(offset); offset += 4;
        const relY = data.readFloatLE(offset); offset += 4;
        const relZ = data.readFloatLE(offset); offset += 4;
        const vehicleMs = data.readUInt32LE(offset); offset += 4;
        const downCount = data.readUInt32LE(offset);

        const gpsData = {
            satellites: { inView: satsInView, used: satsUsed },
            quality: gpsQuality,
            dilutionOfPrecision: { position: pdop},
            position: { latitude: currentLat, longitude: currentLon, altitude: currentAlt },
            homePosition: { latitude: homeLat, longitude: homeLon, altitude: homeAlt },
            lastRTCM,
            accuracy: { threeD: accuracy3D, twoD: accuracy2D },
            vehicleMs,
            downCount
        };

        //console.log('GPS Data:', gpsData);
        this.updateGPSFixType(gpsQuality);
        this.updateRTCMAge(lastRTCM);
        this.updateGPSPanelData(gpsData);
        
        // Update GPS map if available
        if (typeof gpsMap !== 'undefined') {
            gpsMap.updateCurrentPosition(currentLat, currentLon);
            gpsMap.updateHomePosition(homeLat, homeLon);
        }
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
        const yawMisalign = data.readFloatLE(offset); offset += 4;
        const pitchMisalign = data.readFloatLE(offset); offset += 4;
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
            misaligns: { yaw: yawMisalign, roll: pitchMisalign },
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
        };

        // Store latest thrust value
        this.lastThrust = thrust;

        //console.log('Lander Data:', landerData);
        this.updateBatteryVoltage(VBAT);
        this.updatePyroStatus(pyroStatus);
        getControlPlots().updateAttitudeSetpoints(yawSetpoint, pitchSetpoint);
        getControlPlots().updateTargetPosition(landerData.target);
        getControlPlots().updateMisalignments(yawMisalign, pitchMisalign);
        getControlPlots().updateRollMixedCommands(rollMixedYaw, rollMixedPitch);
        
        // Update flight data chart with thrust data
        if (typeof updateFlightChart !== 'undefined') {
            updateFlightChart(this.lastThrust, this.lastAltitude, this.lastTimeSinceLaunch, this.lastVehicleStateValue);
        }
        
        // Update wheel speed indicator with roll command data
        if (typeof wheelSpeedIndicator !== 'undefined') {
            wheelSpeedIndicator.updateWheelSpeed(rollCommand);
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

        //console.log('Kalman Data:', kalmanData);
        
        // Update Kalman standard deviation indicator with uncertainty data
        if (typeof kalmanStdIndicator !== 'undefined') {
            kalmanStdIndicator.updateAllStdDevs(
                kalmanData.uncertainty.position,
                kalmanData.uncertainty.velocity,
                kalmanData.uncertainty.acceleration
            );
        }
    }

    updateDisplaysWithTelemetry(telemetry) {
        // Update global vehicle state if it exists
        
        if (typeof vehicleState !== 'undefined') {
            vehicleState.quaternion = telemetry.quaternion;
            //console.log("Vehicle state updated with telemetry:", telemetry.quaternion);
            // vehicleState.position = telemetry.position;
            // vehicleState.velocity = telemetry.velocity;
            // vehicleState.acceleration = telemetry.acceleration;
        }

        threeScene.updateVehicleTransform();
    }

    updateVehicleState(state) {
        const vehicleStateElement = document.getElementById('vehicle-state');
        if (vehicleStateElement) {
            const stateText = VEHICLE_STATES[state] || `UNKNOWN (${state})`;
            vehicleStateElement.textContent = stateText;
            
            // Update color based on state
            vehicleStateElement.className = 'status-value';
            if (state === 0) {
                vehicleStateElement.classList.add('bad'); // DISARMED
            } else if (state === 1) {
                vehicleStateElement.classList.add('warning'); // ARMED
            } else if (state >= 2 && state <= 7) {
                vehicleStateElement.classList.add('good'); // Active flight states
            } else if (state >= 64 && state <= 69) {
                vehicleStateElement.classList.add('warning'); // Test states
            }
        }
    }

    updateSensorStatus(failmask) {
        const sensorStatusBtn = document.getElementById('sensor-status-btn');
        
        // Update individual sensor statuses
        this.updateIndividualSensor('baro-status', !(failmask & SENSOR_FAIL_BITS.BAROMETER));
        this.updateIndividualSensor('SCH-status', !(failmask & SENSOR_FAIL_BITS.SCH1));
        this.updateIndividualSensor('ICM-status', !(failmask & SENSOR_FAIL_BITS.ICM));
        this.updateIndividualSensor('mag-status', !(failmask & SENSOR_FAIL_BITS.MAGNETOMETER));
        this.updateIndividualSensor('gps-status', !(failmask & SENSOR_FAIL_BITS.GPS));
        
        // Update overall sensor status button
        if (sensorStatusBtn) {
            if (failmask === 0) {
                sensorStatusBtn.textContent = 'ALL GOOD';
                sensorStatusBtn.className = 'sensor-status-btn';
            } else {
                sensorStatusBtn.textContent = 'FAILURES';
                sensorStatusBtn.className = 'sensor-status-btn failure';
            }
        }
    }

    updateIndividualSensor(elementId, isGood) {
        const element = document.getElementById(elementId);
        if (element) {
            element.textContent = isGood ? 'GOOD' : 'FAIL';
            element.className = isGood ? 'sensor-status good' : 'sensor-status failure';
        }
    }

    updateBatteryVoltage(voltage) {
        const batteryElement = document.getElementById('battery-voltage');
        if (batteryElement) {
            batteryElement.textContent = `${voltage.toFixed(1)}V`;
            
            // Color code based on voltage levels
            batteryElement.className = 'status-value';
            if (voltage > 7.0) {
                batteryElement.classList.add('good');
            } else if (voltage > 6.5) {
                batteryElement.classList.add('warning');
            } else {
                batteryElement.classList.add('bad');
            }
        }
    }

    updatePyroStatus(pyroStatus) {
        const pyroElement = document.getElementById('pyro-status');
        if (pyroElement) {
            const isGood = pyroStatus === 127;
            pyroElement.className = isGood ? 'pyro-indicator good' : 'pyro-indicator';
            pyroElement.title = isGood ? 'Good' : 'No Good';
        }
    }

    updateLaunchTime(timeSinceLaunch) {
        const launchTimeElement = document.getElementById('launch-time');
        if (launchTimeElement) {
            if (timeSinceLaunch >= 0) {
                launchTimeElement.textContent = `${timeSinceLaunch.toFixed(1)}s`;
            } else {
                launchTimeElement.textContent = '0s';
            }
        }
    }

    updateGPSFixType(quality) {
        const gpsFixElement = document.getElementById('gps-fix-type');
        if (gpsFixElement) {
            let fixText, className;
            
            switch (quality) {
                case 0:
                    fixText = 'NO FIX';
                    className = 'status-value bad';
                    break;
                case 1:
                    fixText = '3D';
                    className = 'status-value warning';
                    break;
                case 2:
                    fixText = 'DGPS';
                    className = 'status-value warning';
                    break;
                case 4:
                    fixText = 'RTK FIXED';
                    className = 'status-value good';
                    break;
                case 5:
                    fixText = 'RTK FLOAT';
                    className = 'status-value warning';
                    break;
                default:
                    fixText = `UNKNOWN (${quality})`;
                    className = 'status-value bad';
            }
            
            gpsFixElement.textContent = fixText;
            gpsFixElement.className = className;
        }
    }

    updateRTCMAge(lastRTCM) {
        const rtcmAgeElement = document.getElementById('rtcm-age');
        if (rtcmAgeElement) {
            if (lastRTCM > 0) {
                const ageSeconds = (Date.now() - lastRTCM) / 1000;
                rtcmAgeElement.textContent = `${ageSeconds.toFixed(0)}s`;
                
                // Color code based on age
                rtcmAgeElement.className = 'status-value';
                if (ageSeconds < 10) {
                    rtcmAgeElement.classList.add('good');
                } else if (ageSeconds < 30) {
                    rtcmAgeElement.classList.add('warning');
                } else {
                    rtcmAgeElement.classList.add('bad');
                }
            } else {
                rtcmAgeElement.textContent = '--';
                rtcmAgeElement.className = 'status-value';
            }
        }
    }

    updateConnectionStatus(connected, portPath = null, error = null) {
        const statusElement = document.getElementById('serial-status');
        const vehicleStatusElement = document.getElementById('vehicle-status');
        const vehicleStateElement = document.getElementById('vehicle-state');
        
        if (connected) {
            if (statusElement) {
                statusElement.textContent = `Serial: ${portPath || 'CONNECTED'}`;
                statusElement.className = 'serial-status connected';
            }
            if (vehicleStatusElement) {
                vehicleStatusElement.textContent = 'CONNECTED';
            }
            console.log('Serial connected, waiting for telemetry...');
        } else {
            if (statusElement) {
                statusElement.textContent = error ? `Serial: ERROR - ${error}` : 'Serial: DISCONNECTED';
                statusElement.className = 'serial-status';
            }
            if (vehicleStatusElement) {
                vehicleStatusElement.textContent = 'DISCONNECTED';
            }
            if (vehicleStateElement) {
                vehicleStateElement.textContent = 'DISCONNECTED';
                vehicleStateElement.className = 'status-value disconnected';
            }
            this.resetStatusIndicators();
        }
    }

    resetStatusIndicators() {
        // Reset GPS fix
        const gpsFixElement = document.getElementById('gps-fix-type');
        if (gpsFixElement) {
            gpsFixElement.textContent = 'NO FIX';
            gpsFixElement.className = 'status-value bad';
        }
        
        // Reset battery voltage
        const batteryElement = document.getElementById('battery-voltage');
        if (batteryElement) {
            batteryElement.textContent = '0.0V';
            batteryElement.className = 'status-value bad';
        }
        
        // Reset pyro status
        const pyroElement = document.getElementById('pyro-status');
        if (pyroElement) {
            pyroElement.className = 'pyro-indicator';
            pyroElement.title = 'No Good';
        }
        
        // Reset launch time
        const launchTimeElement = document.getElementById('launch-time');
        if (launchTimeElement) {
            launchTimeElement.textContent = '0s';
        }
        
        // Reset RTCM age
        const rtcmAgeElement = document.getElementById('rtcm-age');
        if (rtcmAgeElement) {
            rtcmAgeElement.textContent = '--';
            rtcmAgeElement.className = 'status-value';
        }
        
        // Reset sensor status - default all to failed
        this.updateIndividualSensor('baro-status', false);
        this.updateIndividualSensor('SCH-status', false);
        this.updateIndividualSensor('ICM-status', false);
        this.updateIndividualSensor('mag-status', false);
        this.updateIndividualSensor('gps-status', false);
        
        const sensorStatusBtn = document.getElementById('sensor-status-btn');
        if (sensorStatusBtn) {
            sensorStatusBtn.textContent = 'FAILURES';
            sensorStatusBtn.className = 'sensor-status-btn failure';
        }

        lastTelemetryTime = 0;
        const telemetryAgeElement = document.getElementById('telemetry-age');
        if (telemetryAgeElement) {
            telemetryAgeElement.textContent = '--';
            telemetryAgeElement.className = 'status-value bad';
        }

        // Reset sensor panel data
        this.resetSensorPanelData();
        
        // Reset flight chart
        if (typeof clearFlightChart !== 'undefined') {
            clearFlightChart();
        }
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

    // Pack a command packet according to LFS LINK-80 protocol
    packCommandPacket(commandType) {
        // Increment command message ID (wraps at 255)
        this.commandMessageId = (this.commandMessageId + 1) & 0xFF;
        
        // Create payload: [reserved_byte][command_type][message_id]
        let offset = 0;
        this.payloadBuffer[offset++] = 0; // Reserved addressing byte
        this.payloadBuffer[offset++] = commandType; // Command type enum
        this.payloadBuffer[offset++] = this.commandMessageId; // Message ID
        
        const payloadSize = 3;
        const packetSize = this.packPacket(this.payloadBuffer, payloadSize);
        
        if (packetSize > 0) {
            return this.packetBuffer.slice(0, packetSize);
        }
        return null;
    }

    // Pack a complete LFS LINK-80 packet with COBS encoding and CRC
    packPacket(payload, payloadSize) {
        if (payloadSize > 245) { // MAX_PACKET_SIZE - HEADER_SIZE - CHECKSUM_SIZE
            return 0;
        }
        
        // Calculate CRC32 of original payload
        const crc = this.calculateCRC32(payload.slice(0, payloadSize));
        
        // Create [payload + checksum] section
        payload.copy(this.payloadAndChecksumBuffer, 0, 0, payloadSize);
        
        // Append checksum in little endian format (consistent with protocol)
        this.payloadAndChecksumBuffer.writeUInt32LE(crc, payloadSize);
        
        const totalDataSize = payloadSize + CHECKSUM_SIZE;
        const requiredSize = HEADER_SIZE + totalDataSize;
        
        if (requiredSize > MAX_PACKET_SIZE) {
            return 0;
        }
        
        // Find COBS offset (first occurrence of 0xAA in [payload + checksum])
        let cobsOffset = 0;
        for (let i = 0; i < totalDataSize; i++) {
            if (this.payloadAndChecksumBuffer[i] === PACKET_HEADER) {
                cobsOffset = i;
                break;
            }
        }
        
        // Apply COBS encoding if needed
        if (cobsOffset > 0) {
            this.applyCOBSEncoding(this.payloadAndChecksumBuffer, totalDataSize);
        }
        
        // Build final packet
        let packetPos = 0;
        this.packetBuffer[packetPos++] = PACKET_HEADER;
        this.packetBuffer[packetPos++] = totalDataSize;
        this.packetBuffer[packetPos++] = cobsOffset;
        
        // Add COBS-encoded [payload + checksum]
        this.payloadAndChecksumBuffer.copy(this.packetBuffer, packetPos, 0, totalDataSize);
        packetPos += totalDataSize;
        
        return packetPos;
    }

    // Apply COBS encoding to buffer
    applyCOBSEncoding(data, dataSize) {
        let currentPos = 0;
        
        while (currentPos < dataSize) {
            if (data[currentPos] === PACKET_HEADER) {
                // Find next 0xAA or end of data
                let nextAA = currentPos + 1;
                while (nextAA < dataSize && data[nextAA] !== PACKET_HEADER) {
                    nextAA++;
                }
                
                if (nextAA < dataSize) {
                    // Replace current 0xAA with offset to next 0xAA
                    data[currentPos] = nextAA - currentPos;
                } else {
                    // No more 0xAA bytes, replace with 0
                    data[currentPos] = 0;
                }
            }
            currentPos++;
        }
    }

    // Convert buffer to hex string for inspection
    bufferToHex(buffer) {
        return Array.from(buffer, byte => byte.toString(16).padStart(2, '0').toUpperCase()).join(' ');
    }

    // Send a command to the vehicle
    async sendCommand(commandType) {
        const packet = this.packCommandPacket(commandType);
        if (packet) {
            const hexString = this.bufferToHex(packet);
            console.log(`Sending command ${commandType}, message ID ${this.commandMessageId}, packet size: ${packet.length}`);
            console.log(`Command packet hex: ${hexString}`);
            return await this.sendData(packet);
        } else {
            console.error('Failed to create command packet');
            return false;
        }
    }

    // RTCM Message Packing Functions
    
    // Pack and send RTCM data as LFS LINK-80 packets
    async sendRTCMData(rtcmData) {
        if (!this.rtcmEnabled || !rtcmData || rtcmData.length === 0) {
            return false;
        }

        // Get unique RTCM ID (0-127)
        const rtcmId = this.getNextRTCMId();
        
        // Maximum payload size per packet = 245 bytes (MAX_PACKET_SIZE - HEADER_SIZE - CHECKSUM_SIZE)
        // Actual payload = reserved (1) + message_type (1) + rtcm_id (1) + rtcm_fragment_data
        const maxFragmentSize = 242; // 245 - 3 for reserved/message_type/rtcm_id
        
        if (rtcmData.length <= maxFragmentSize) {
            // Single packet - use message type 55 (RTCM_FINAL)
            return await this.sendRTCMFragment(rtcmId, rtcmData, MESSAGE_TYPES.RTCM_FINAL);
        } else {
            // Multiple packets needed
            const fragments = this.splitRTCMData(rtcmData, maxFragmentSize);
            const messageTypes = [
                MESSAGE_TYPES.RTCM_FRAGMENT_1,
                MESSAGE_TYPES.RTCM_FRAGMENT_2,
                MESSAGE_TYPES.RTCM_FRAGMENT_3,
                MESSAGE_TYPES.RTCM_FRAGMENT_4,
                MESSAGE_TYPES.RTCM_FINAL
            ];
            
            // Send all fragments
            for (let i = 0; i < fragments.length; i++) {
                const messageType = i < messageTypes.length - 1 ? messageTypes[i] : MESSAGE_TYPES.RTCM_FINAL;
                const success = await this.sendRTCMFragment(rtcmId, fragments[i], messageType);
                if (!success) {
                    console.error(`Failed to send RTCM fragment ${i + 1}/${fragments.length}`);
                    return false;
                }
            }
            
            console.log(`Successfully sent RTCM data in ${fragments.length} fragments (ID: ${rtcmId})`);
            return true;
        }
    }
    
    // Split RTCM data into fragments
    splitRTCMData(data, maxFragmentSize) {
        const fragments = [];
        let offset = 0;
        
        while (offset < data.length) {
            const remainingBytes = data.length - offset;
            const fragmentSize = Math.min(maxFragmentSize, remainingBytes);
            const fragment = data.slice(offset, offset + fragmentSize);
            fragments.push(fragment);
            offset += fragmentSize;
        }
        
        return fragments;
    }
    
    // Send a single RTCM fragment
    async sendRTCMFragment(rtcmId, fragmentData, messageType) {
        // Create payload: [reserved_byte][message_type][rtcm_id][rtcm_fragment_data]
        let offset = 0;
        this.payloadBuffer[offset++] = 0; // Reserved addressing byte
        this.payloadBuffer[offset++] = messageType; // Message type (51-55)
        this.payloadBuffer[offset++] = rtcmId; // RTCM ID (0-127)
        
        // Copy fragment data
        fragmentData.copy(this.payloadBuffer, offset);
        offset += fragmentData.length;
        
        const payloadSize = offset;
        
        // Use existing packPacket method (same as commands)
        const packetSize = this.packPacket(this.payloadBuffer, payloadSize);
        
        if (packetSize > 0) {
            const packet = this.packetBuffer.slice(0, packetSize);
            const hexString = this.bufferToHex(packet);
            console.log(`Sending RTCM fragment (ID: ${rtcmId}, type: ${messageType}, size: ${fragmentData.length})`);
            console.log(`RTCM packet hex: ${hexString}`);
            return await this.sendData(packet);
        } else {
            console.error('Failed to create RTCM packet');
            return false;
        }
    }
    
    // Get next RTCM ID (0-127, wrapping)
    getNextRTCMId() {
        this.rtcmIdCounter = (this.rtcmIdCounter + 1) % 128;
        return this.rtcmIdCounter;
    }
    
    // Enable/disable RTCM functionality
    setRTCMEnabled(enabled) {
        this.rtcmEnabled = enabled;
        console.log('RTCM functionality', enabled ? 'enabled' : 'disabled');
    }

    // Test RTCM packet creation
    testRTCMPacketCreation() {
        // Create a test RTCM message
        const testRTCM = Buffer.from([0xD3, 0x00, 0x13, 0x3E, 0xD0, 0x00, 0x03, 0x8A, 0x24, 0x61, 0x61, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1C, 0x9F]);
        
        console.log('Testing RTCM packet creation...');
        console.log('Test RTCM data length:', testRTCM.length);
        console.log('Test RTCM hex:', this.bufferToHex(testRTCM));
        
        this.sendRTCMData(testRTCM);
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

    updateSensorPanelData(sensorData) {
        // Update gyroscope data
        const gyroYawEl = document.getElementById('gyro-yaw');
        const gyroPitchEl = document.getElementById('gyro-pitch');
        const gyroRollEl = document.getElementById('gyro-roll');
        
        if (gyroYawEl) gyroYawEl.textContent = sensorData.gyro.yaw.toFixed(3);
        if (gyroPitchEl) gyroPitchEl.textContent = sensorData.gyro.pitch.toFixed(3);
        if (gyroRollEl) gyroRollEl.textContent = sensorData.gyro.roll.toFixed(3);

        // Update gyroscope bias data
        const gyroBiasYawEl = document.getElementById('gyro-bias-yaw');
        const gyroBiasPitchEl = document.getElementById('gyro-bias-pitch');
        const gyroBiasRollEl = document.getElementById('gyro-bias-roll');
        
        if (gyroBiasYawEl) gyroBiasYawEl.textContent = sensorData.gyroBias.yaw.toFixed(5);
        if (gyroBiasPitchEl) gyroBiasPitchEl.textContent = sensorData.gyroBias.pitch.toFixed(5);
        if (gyroBiasRollEl) gyroBiasRollEl.textContent = sensorData.gyroBias.roll.toFixed(5);

        // Update accelerometer data
        const accelXEl = document.getElementById('accel-x');
        const accelYEl = document.getElementById('accel-y');
        const accelZEl = document.getElementById('accel-z');
        
        if (accelXEl) accelXEl.textContent = sensorData.accelerometer.x.toFixed(3);
        if (accelYEl) accelYEl.textContent = sensorData.accelerometer.y.toFixed(3);
        if (accelZEl) accelZEl.textContent = sensorData.accelerometer.z.toFixed(3);

        // Update barometer data
        const baroAltEl = document.getElementById('baro-altitude');
        if (baroAltEl) baroAltEl.textContent = sensorData.baroAltitude.toFixed(2);

        // Update SD card status
        const sdStatusEl = document.getElementById('sd-status');
        if (sdStatusEl) {
            sdStatusEl.textContent = sensorData.sdGood ? 'GOOD' : 'FAIL';
            sdStatusEl.className = sensorData.sdGood ? 'sensor-status good' : 'sensor-status failure';
        }
    }

    updateGPSPanelData(gpsData) {
        // Update satellite information
        const gpsSatsEl = document.getElementById('gps-sats');
        if (gpsSatsEl) {
            gpsSatsEl.textContent = `${gpsData.satellites.used}/${gpsData.satellites.inView}`;
        }

        // Update PDOP
        const gpsPdopEl = document.getElementById('gps-pdop');
        if (gpsPdopEl) {
            gpsPdopEl.textContent = gpsData.dilutionOfPrecision.position.toFixed(2);
        }

        // Update 3D accuracy
        const gps3dAccEl = document.getElementById('gps-3d-acc');
        if (gps3dAccEl) {
            gps3dAccEl.textContent = `${gpsData.accuracy.threeD.toFixed(2)}m`;
        }

        // Update 2D accuracy
        const gps2dAccEl = document.getElementById('gps-2d-acc');
        if (gps2dAccEl) {
            gps2dAccEl.textContent = `${gpsData.accuracy.twoD.toFixed(2)}m`;
        }

        // Update current GPS position (if elements exist from sensor panel)
        const currentLatEl = document.getElementById('current-lat');
        const currentLonEl = document.getElementById('current-lon');
        const currentAltEl = document.getElementById('current-alt');
        
        if (currentLatEl) currentLatEl.textContent = gpsData.position.latitude.toFixed(6);
        if (currentLonEl) currentLonEl.textContent = gpsData.position.longitude.toFixed(6);
        if (currentAltEl) currentAltEl.textContent = gpsData.position.altitude.toFixed(2);

        // Update home GPS position (if elements exist from sensor panel)
        const homeLatEl = document.getElementById('home-lat');
        const homeLonEl = document.getElementById('home-lon');
        const homeAltEl = document.getElementById('home-alt');
        
        if (homeLatEl) homeLatEl.textContent = gpsData.homePosition.latitude.toFixed(6);
        if (homeLonEl) homeLonEl.textContent = gpsData.homePosition.longitude.toFixed(6);
        if (homeAltEl) homeAltEl.textContent = gpsData.homePosition.altitude.toFixed(2);
    }

    resetSensorPanelData() {
        // Reset gyroscope data
        const gyroElements = ['gyro-yaw', 'gyro-pitch', 'gyro-roll'];
        gyroElements.forEach(id => {
            const el = document.getElementById(id);
            if (el) el.textContent = '0.000';
        });

        // Reset gyroscope bias data
        const gyroBiasElements = ['gyro-bias-yaw', 'gyro-bias-pitch', 'gyro-bias-roll'];
        gyroBiasElements.forEach(id => {
            const el = document.getElementById(id);
            if (el) el.textContent = '0.00000';
        });

        // Reset accelerometer data
        const accelElements = ['accel-x', 'accel-y', 'accel-z'];
        accelElements.forEach(id => {
            const el = document.getElementById(id);
            if (el) el.textContent = '0.000';
        });

        // Reset barometer data
        const baroAltEl = document.getElementById('baro-altitude');
        if (baroAltEl) baroAltEl.textContent = '0.00';

        // Reset GPS position data
        const gpsElements = ['current-lat', 'current-lon', 'current-alt', 'home-lat', 'home-lon', 'home-alt'];
        gpsElements.forEach(id => {
            const el = document.getElementById(id);
            if (el) {
                if (id.includes('lat') || id.includes('lon')) {
                    el.textContent = '0.000000';
                } else {
                    el.textContent = '0.00';
                }
            }
        });

        // Reset GPS panel data
        const gpsPanelElements = ['gps-sats', 'gps-pdop', 'gps-3d-acc', 'gps-2d-acc'];
        gpsPanelElements.forEach(id => {
            const el = document.getElementById(id);
            if (el) {
                switch(id) {
                    case 'gps-sats':
                        el.textContent = '0/0';
                        break;
                    case 'gps-pdop':
                        el.textContent = '--';
                        break;
                    case 'gps-3d-acc':
                    case 'gps-2d-acc':
                        el.textContent = '--';
                        break;
                }
            }
        });

        // Reset SD card status
        const sdStatusEl = document.getElementById('sd-status');
        if (sdStatusEl) {
            sdStatusEl.textContent = 'FAIL';
            sdStatusEl.className = 'sensor-status failure';
        }

        // Reset velocity indicator
        if (typeof velocityIndicator !== 'undefined') {
            velocityIndicator.updateVelocity(0);
        }
    }
}

// Global instance
const serialComm = new LFSSerialComm();

// Initialize status indicators on startup
document.addEventListener('DOMContentLoaded', () => {
    serialComm.resetStatusIndicators();
    telemetryUpdateInterval = setInterval(updateTelemetryAge, 10);
});

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
    console.log('Command requested:', command);
    
    // Map command strings to command types
    let commandType = null;
    switch (command) {
        case 'PING':
            commandType = COMMAND_TYPES.PING;
            break;
        case 'ARM':
            commandType = COMMAND_TYPES.ARM;
            break;
        case 'DISARM':
            commandType = COMMAND_TYPES.DISARM;
            break;
        case 'wheeltest':
            commandType = COMMAND_TYPES.WHEEL_TEST;
            break;
        case 'stabtest':
            commandType = COMMAND_TYPES.STAB_TEST;
            break;
        case 'positest':
            commandType = COMMAND_TYPES.POS_TEST;
            break;
        case 'trajtest':
            commandType = COMMAND_TYPES.TRAJ_TEST;
            break;
        case 'misalignTest':
            commandType = COMMAND_TYPES.MISALIGN_TEST;
            break;
        case 'preptest':
            commandType = COMMAND_TYPES.TEST_PREP;
            break;
        case 'pyroTest':
            commandType = COMMAND_TYPES.PYRO_TEST;
            break;
        default:
            console.error('Unknown command:', command);
            return false;
    }
    
    if (serialComm.isConnected && commandType !== null) {
        try {
            const success = await serialComm.sendCommand(commandType);
            if (success) {
                console.log(`Command ${command} (type ${commandType}) sent successfully`);
            } else {
                console.error(`Failed to send command ${command}`);
            }
            return success;
        } catch (error) {
            console.error('Error sending command:', error);
            return false;
        }
    } else if (!serialComm.isConnected) {
        console.warn('Cannot send command: not connected to serial port');
        return false;
    }
    
    return false;
}

// Global test functions
function testRTCMPackets() {
    console.log('Testing RTCM packet creation...');
    return serialComm.testRTCMPacketCreation();
}

module.exports = { LFSSerialComm, serialComm, connectSerial, sendCommand, testRTCMPackets };