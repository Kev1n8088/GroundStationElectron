// Simulation functionality
class Simulation {
    constructor() {
        this.isRunning = false;
        this.time = 0;
        this.timeStep = 0.05; // 50ms updates
        this.intervalId = null;
        
        // Simulation parameters
        this.flightPattern = 'circle'; // 'circle', 'figure8', 'waypoint', 'manual'
        this.baseAltitude = 10;
        this.flightRadius = 8;
        this.speed = 0.5;
        
        // Waypoint mode
        this.waypoints = [
            { x: 0, y: 10, z: 0 },
            { x: 10, y: 12, z: 5 },
            { x: 5, y: 8, z: -8 },
            { x: -8, y: 15, z: 3 },
            { x: 0, y: 10, z: 0 }
        ];
        this.currentWaypoint = 0;
        this.waypointThreshold = 1.0;
    }

    start() {
        if (this.isRunning) return;
        
        this.isRunning = true;
        this.intervalId = setInterval(() => this.update(), this.timeStep * 1000);
        console.log('Simulation started');
    }

    stop() {
        if (!this.isRunning) return;
        
        this.isRunning = false;
        if (this.intervalId) {
            clearInterval(this.intervalId);
            this.intervalId = null;
        }
        console.log('Simulation stopped');
    }

    toggle() {
        if (this.isRunning) {
            this.stop();
        } else {
            this.start();
        }
    }

    update() {
        this.time += this.timeStep;
        
        switch (this.flightPattern) {
            case 'circle':
                this.updateCirclePattern();
                break;
            case 'figure8':
                this.updateFigure8Pattern();
                break;
            case 'waypoint':
                this.updateWaypointPattern();
                break;
            case 'manual':
                this.updateManualPattern();
                break;
        }

        
        // Add some realistic noise and dynamics
        this.addRealisticMovement();

        this.updateOneAxis(); // For one-axis test
        
        this.updateQuatFromEuler();
        
        // Update displays
        this.updateDisplays();
    }

    updateOneAxis(){
        const angle = this.time * this.speed;
        vehicleState.euler.roll = 0; // Roll oscillation
        vehicleState.euler.pitch = 0; // correct direction
        vehicleState.euler.yaw = 0; // correct rotation direction, right hand for z axis

        vehicleState.position.x = 0; // side to side, pitch axis. Flipped
        vehicleState.position.y = 0; // vertical, roll axis
        vehicleState.position.z = 0;

    }

    updateQuatFromEuler() {
        // needs to be fixed I think w is flipped
        const { roll, pitch, yaw } = vehicleState.euler;
    
        // Convert degrees to radians if needed
        const toRad = angle => angle * Math.PI / 180;
        const r = toRad(roll);
        const p = toRad(pitch);
        const y = toRad(yaw);
    
        const cy = Math.cos(y / 2);
        const cp = Math.cos(p / 2);
        const cr = Math.cos(r / 2);
    
        const sy = Math.sin(y / 2);
        const sp = Math.sin(p / 2);
        const sr = Math.sin(r / 2);
    
        // Quaternion: w, x, y, z
        const w = cr * cp * cy + sr * sp * sy;
        const x = sr * cp * cy - cr * sp * sy;
        const yq = cr * sp * cy + sr * cp * sy;
        const z = cr * cp * sy - sr * sp * cy;
    
        vehicleState.quaternion = {x, y: yq, z, w };
        vehicleState.quaternion = {x: 0, y: 0, z: 0, w:1};
    }

    updateCirclePattern() {
        const angle = this.time * this.speed;
        
        vehicleState.position.x = Math.sin(angle) * this.flightRadius;
        vehicleState.position.z = Math.cos(angle) * this.flightRadius;
        vehicleState.position.y = this.baseAltitude + Math.sin(this.time * 0.3) * 2;
        
        // Banking turn
        vehicleState.euler.roll = -Math.sin(angle) * 15;
        vehicleState.euler.pitch = Math.cos(this.time * 0.8) * 5;
        vehicleState.euler.yaw = (angle * 180 / Math.PI) % 360;


    }

    updateFigure8Pattern() {
        const t = this.time * this.speed;
        const scale = this.flightRadius;
        
        // Lissajous curve for figure-8
        vehicleState.position.x = Math.sin(t) * scale;
        vehicleState.position.z = Math.sin(2 * t) * scale * 0.5;
        vehicleState.position.y = this.baseAltitude + Math.sin(this.time * 0.4) * 1.5;
        
        // Calculate banking based on turn rate
        const dx = Math.cos(t) * scale * this.speed;
        const dz = Math.cos(2 * t) * scale * this.speed;
        const turnRate = Math.atan2(dz, dx);
        
        vehicleState.euler.roll = Math.sin(turnRate * 2) * 20;
        vehicleState.euler.pitch = Math.cos(this.time * 0.6) * 3;
        vehicleState.euler.yaw = (turnRate * 180 / Math.PI) % 360;
    }

    updateWaypointPattern() {
        const target = this.waypoints[this.currentWaypoint];
        const current = vehicleState.position;
        
        // Calculate distance to target
        const dx = target.x - current.x;
        const dy = target.y - current.y;
        const dz = target.z - current.z;
        const distance = Math.sqrt(dx*dx + dy*dy + dz*dz);
        
        if (distance < this.waypointThreshold) {
            // Move to next waypoint
            this.currentWaypoint = (this.currentWaypoint + 1) % this.waypoints.length;
            console.log(`Reached waypoint ${this.currentWaypoint}, moving to next`);
        } else {
            // Move toward current waypoint
            const moveSpeed = this.speed * 2;
            const factor = Math.min(moveSpeed / distance, 1.0);
            
            vehicleState.position.x += dx * factor;
            vehicleState.position.y += dy * factor;
            vehicleState.position.z += dz * factor;
            
            // Point toward target
            vehicleState.euler.yaw = Math.atan2(dx, dz) * 180 / Math.PI;
            vehicleState.euler.pitch = Math.atan2(-dy, Math.sqrt(dx*dx + dz*dz)) * 180 / Math.PI;
            vehicleState.euler.roll = Math.sin(this.time * 2) * 10; // Banking in turns
        }
    }

    updateManualPattern() {
        // Gentle drift and hover simulation
        vehicleState.position.x += Math.sin(this.time * 0.2) * 0.02;
        vehicleState.position.z += Math.cos(this.time * 0.15) * 0.02;
        vehicleState.position.y += Math.sin(this.time * 0.3) * 0.01;
        
        // Small attitude adjustments
        vehicleState.euler.roll += (Math.random() - 0.5) * 0.5;
        vehicleState.euler.pitch += (Math.random() - 0.5) * 0.5;
        vehicleState.euler.yaw += (Math.random() - 0.5) * 1.0;
    }

    addRealisticMovement() {
        // Add some turbulence/noise
        const turbulenceStrength = 0.1;
        
        vehicleState.euler.roll += (Math.random() - 0.5) * turbulenceStrength;
        vehicleState.euler.pitch += (Math.random() - 0.5) * turbulenceStrength;
        vehicleState.euler.yaw += (Math.random() - 0.5) * turbulenceStrength * 0.5;
        
        // Add altitude variation
        vehicleState.position.y += (Math.random() - 0.5) * 0.05;
        
        // Clamp values to realistic ranges
        vehicleState.euler.roll = Math.max(-45, Math.min(45, vehicleState.euler.roll));
        vehicleState.euler.pitch = Math.max(-30, Math.min(30, vehicleState.euler.pitch));
        vehicleState.position.y = Math.max(0.5, vehicleState.position.y);
        
        // Normalize yaw to 0-360
        while (vehicleState.euler.yaw < 0) vehicleState.euler.yaw += 360;
        while (vehicleState.euler.yaw >= 360) vehicleState.euler.yaw -= 360;
    }

    updateDisplays() {
        // Update 3D scene
        if (threeScene && threeScene.initialized) {
            threeScene.updateVehicleTransform();
        }
        
        // Update status displays
        document.getElementById('pos-x').textContent = vehicleState.position.x.toFixed(1);
        document.getElementById('pos-y').textContent = vehicleState.position.y.toFixed(1);
        document.getElementById('pos-z').textContent = vehicleState.position.z.toFixed(1);
        document.getElementById('roll').textContent = vehicleState.euler.roll.toFixed(1);
        document.getElementById('pitch').textContent = vehicleState.euler.pitch.toFixed(1);
        document.getElementById('yaw').textContent = vehicleState.euler.yaw.toFixed(1);
        
        // Update charts
        if (chartManager && chartManager.initialized) {
            chartManager.updateCharts();
        }
    }

    setFlightPattern(pattern) {
        if (['circle', 'figure8', 'waypoint', 'manual'].includes(pattern)) {
            this.flightPattern = pattern;
            console.log('Flight pattern changed to:', pattern);
        }
    }

    setPosition(x, y, z) {
        vehicleState.position.x = x;
        vehicleState.position.y = y;
        vehicleState.position.z = z;
        this.updateDisplays();
    }

    setAttitude(roll, pitch, yaw) {
        vehicleState.euler.roll = roll;
        vehicleState.euler.pitch = pitch;
        vehicleState.euler.yaw = yaw;
        this.updateDisplays();
    }

    // Emergency procedures
    returnToHome() {
        console.log('Return to home initiated');
        this.flightPattern = 'waypoint';
        this.waypoints = [
            { ...vehicleState.position }, // Current position
            { x: 0, y: vehicleState.position.y, z: 0 }, // Home at current altitude
            { x: 0, y: 5, z: 0 } // Land at home
        ];
        this.currentWaypoint = 0;
    }

    emergencyLand() {
        console.log('Emergency landing initiated');
        this.flightPattern = 'waypoint';
        this.waypoints = [
            { ...vehicleState.position },
            { x: vehicleState.position.x, y: 0.5, z: vehicleState.position.z }
        ];
        this.currentWaypoint = 0;
    }

    // Add custom waypoint
    addWaypoint(x, y, z) {
        this.waypoints.push({ x, y, z });
        console.log('Waypoint added:', { x, y, z });
    }

    clearWaypoints() {
        this.waypoints = [];
        this.currentWaypoint = 0;
        console.log('Waypoints cleared');
    }

    // Get simulation status
    getStatus() {
        return {
            isRunning: this.isRunning,
            time: this.time,
            pattern: this.flightPattern,
            currentWaypoint: this.currentWaypoint,
            totalWaypoints: this.waypoints.length,
            vehicleState: { ...vehicleState }
        };
    }
}

// Global instance
const simulation = new Simulation();

// Global functions for HTML interface
function toggleSimulation() {
    simulation.toggle();
    
    // Update button text
    const button = event.target;
    if (simulation.isRunning) {
        button.textContent = 'Stop Simulation';
        button.style.backgroundColor = '#ff4444';
    } else {
        button.textContent = 'Start Simulation';
        button.style.backgroundColor = '';
    }
}

function setPosition() {
    const x = parseFloat(document.getElementById('manual-x').value) || 0;
    const y = parseFloat(document.getElementById('manual-y').value) || 0;
    const z = parseFloat(document.getElementById('manual-z').value) || 0;
    
    simulation.setPosition(x, y, z);
    console.log('Manual position set:', { x, y, z });
}

// Additional control functions
function setFlightPattern(pattern) {
    simulation.setFlightPattern(pattern);
}

function returnToHome() {
    simulation.returnToHome();
}

function emergencyLand() {
    simulation.emergencyLand();
}