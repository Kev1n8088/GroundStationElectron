class ControlPlots {
    constructor() {
        this.positionCanvas = document.getElementById('position-plot');
        this.attitudeCanvas = document.getElementById('attitude-plot');
        this.gimbalCanvas = document.getElementById('gimbal-plot');
        
        this.positionCtx = this.positionCanvas.getContext('2d');
        this.attitudeCtx = this.attitudeCanvas.getContext('2d');
        this.gimbalCtx = this.gimbalCanvas.getContext('2d');
        
        // Plot parameters - adjusted for limited ranges
        this.positionScale = 100; // pixels per meter (±1m range)
        this.attitudeScale = 200; // pixels per radian (±30° range)
        this.gimbalScale = 1146; // pixels per radian (±5° range, optimized for canvas utilization)
        
        // Data storage
        this.currentPosition = { x: 0, y: 0, z: 0 };
        this.targetPosition = { y: 0, z: 0 };
        this.positionError = { y: 0, z: 0 };
        this.quaternion = { w: 1, x: 0, y: 0, z: 0 };
        this.attitudeSetpoints = { yaw: 0, pitch: 0 };
        
        // Gimbal plot data
        this.misalignments = { yaw: 0, pitch: 0 };
        this.rollMixedCommands = { yaw: 0, pitch: 0 };
        
        this.initializeCanvases();
        this.startAnimation();
    }
    
    initializeCanvases() {
        this.resizeCanvases();
        window.addEventListener('resize', () => this.resizeCanvases());
    }
    
    resizeCanvases() {
        // Calculate available height for plots (container minus title and padding)
        const containerHeight = this.positionCanvas.parentElement.parentElement.clientHeight;
        const availableHeight = containerHeight - 10; // Account for title and padding
        
        // Make plots square - use the smaller of available width/height
        const plotSize = Math.min(400, availableHeight - 20);
        
        // Set all canvases to be square
        [this.positionCanvas, this.attitudeCanvas, this.gimbalCanvas].forEach(canvas => {
            canvas.style.width = plotSize + 'px';
            canvas.style.height = plotSize + 'px';
            canvas.width = plotSize;
            canvas.height = plotSize;
        });
    }
    
    quaternionToEuler(q) {
        const sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
        const cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
        const roll = Math.atan2(sinr_cosp, cosr_cosp);
        
        const sinp = 2 * (q.w * q.y - q.z * q.x);
        let pitch;
        if (Math.abs(sinp) >= 1) {
            pitch = Math.sign(sinp) * Math.PI / 2;
        } else {
            pitch = Math.asin(sinp);
        }
        
        const siny_cosp = 2 * (q.w * q.z + q.x * q.y);
        const cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
        const yaw = Math.atan2(siny_cosp, cosy_cosp);
        
        return { roll, pitch, yaw };
    }
    
    // Quaternion multiplication: q1 * q2
    quaternionMultiply(q1, q2) {
        return {
            w: q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z,
            x: q1.w * q2.x + q1.x * q2.w + q1.y * q2.z - q1.z * q2.y,
            y: q1.w * q2.y - q1.x * q2.z + q1.y * q2.w + q1.z * q2.x,
            z: q1.w * q2.z + q1.x * q2.y - q1.y * q2.x + q1.z * q2.w
        };
    }
    
    // Quaternion conjugate
    quaternionConj(q) {
        return { w: q.w, x: -q.x, y: -q.y, z: -q.z };
    }
    
    // Rotate a 3D vector by a quaternion (body to world frame)
    rotateVectorByQuaternion(vector, quaternion) {
        // Convert vector to quaternion (w=0, x=vector.x, y=vector.y, z=vector.z)
        const vecQuat = { w: 0, x: vector.x, y: vector.y, z: vector.z };
        
        // Rotation: q * v * q_conj (but since we're body-to-world, just q * v * q_conj)
        const temp = this.quaternionMultiply(quaternion, vecQuat);
        const result = this.quaternionMultiply(temp, this.quaternionConj(quaternion));
        
        return { x: result.x, y: result.y, z: result.z };
    }
    
    drawPositionPlot() {
        const ctx = this.positionCtx;
        const canvas = this.positionCanvas;
        const centerX = canvas.width / 2;
        const centerY = canvas.height / 2;
        
        // Clear canvas
        ctx.clearRect(0, 0, canvas.width, canvas.height);
        
        // Draw grid with units
        this.drawGridWithUnits(ctx, canvas, centerX, centerY, this.positionScale, 'm');
        
        // Draw center circle (lander - always at center)
        ctx.fillStyle = '#00ff00';
        ctx.beginPath();
        ctx.arc(centerX, centerY, 8, 0, 2 * Math.PI);
        ctx.fill();
        
        // Draw line from center to error position
        const errorX = centerX + this.positionError.y * this.positionScale;
        const errorY = centerY - this.positionError.z * this.positionScale; // Y-axis flipped
        
        ctx.strokeStyle = '#ff6600';
        ctx.lineWidth = 3;
        ctx.beginPath();
        ctx.moveTo(centerX, centerY);
        ctx.lineTo(errorX, errorY);
        ctx.stroke();
        
        // Draw target indicator at line end
        ctx.fillStyle = '#ff6600';
        ctx.beginPath();
        ctx.arc(errorX, errorY, 5, 0, 2 * Math.PI);
        ctx.fill();
        
        // Add axis labels
        ctx.fillStyle = '#aaa';
        ctx.font = '12px Arial';
        ctx.fillText('Y (m)', canvas.width - 35, centerY - 5);
        ctx.fillText('Z (m)', centerX + 5, 15);
    }
    
    drawAttitudePlot() {
        const ctx = this.attitudeCtx;
        const canvas = this.attitudeCanvas;
        const centerX = canvas.width / 2;
        const centerY = canvas.height / 2;
        
        // Clear canvas
        ctx.clearRect(0, 0, canvas.width, canvas.height);
        
        // Draw grid with units (degrees)
        this.drawGridWithUnits(ctx, canvas, centerX, centerY, this.attitudeScale, '°');
        
        // Convert quaternion to euler angles
        const euler = this.quaternionToEuler(this.quaternion);
        
        // Draw setpoint circle (yaw = horizontal FLIPPED, pitch = vertical)
        const setpointX = centerX + this.attitudeSetpoints.yaw * this.attitudeScale;
        const setpointY = centerY + this.attitudeSetpoints.pitch * this.attitudeScale;
        
        ctx.fillStyle = '#0099ff';
        ctx.beginPath();
        ctx.arc(setpointX, setpointY, 8, 0, 2 * Math.PI);
        ctx.fill();
        
        // Draw line from center to current attitude
        const currentX = centerX + euler.yaw * this.attitudeScale; 
        const currentY = centerY + euler.pitch * this.attitudeScale;
        
        ctx.strokeStyle = '#ff3366';
        ctx.lineWidth = 3;
        ctx.beginPath();
        ctx.moveTo(centerX, centerY);
        ctx.lineTo(currentX, currentY);
        ctx.stroke();
        
        // Draw center point
        ctx.fillStyle = '#666';
        ctx.beginPath();
        ctx.arc(centerX, centerY, 4, 0, 2 * Math.PI);
        ctx.fill();
        
        // Add axis labels
        ctx.fillStyle = '#aaa';
        ctx.font = '12px Arial';
        ctx.fillText('Yaw (°)', canvas.width - 45, centerY - 5);
        ctx.fillText('Pitch (°)', centerX + 5, 15);
    }
    
    drawGridWithUnits(ctx, canvas, centerX, centerY, scale, unit) {
        ctx.strokeStyle = '#333';
        ctx.lineWidth = 1;
        
        // Draw center lines
        ctx.strokeStyle = '#555';
        ctx.lineWidth = 2;
        ctx.beginPath();
        ctx.moveTo(centerX, 0);
        ctx.lineTo(centerX, canvas.height);
        ctx.moveTo(0, centerY);
        ctx.lineTo(canvas.width, centerY);
        ctx.stroke();
        
        // Determine grid spacing and tick positions based on unit type
        let gridSpacing, tickPositions;
        if (unit === 'm') {
            // For position: ±1m range with 0.5m ticks
            gridSpacing = scale * 0.5; // 0.5m spacing
            tickPositions = [-1, -0.5, 0.5, 1]; // Show -1m, -0.5m, +0.5m, +1m
        } else { // degrees
            // For attitude: ±30° range with 15° ticks
            gridSpacing = scale * (Math.PI / 180) * 15; // 15 degree spacing
            tickPositions = [-30, -15, 15, 30]; // Show -30°, -15°, +15°, +30°
        }
        
        ctx.strokeStyle = '#333';
        ctx.lineWidth = 1;
        
        // Draw grid lines and labels for specific tick positions
        ctx.fillStyle = '#666';
        ctx.font = '10px Arial';
        ctx.textAlign = 'center';
        
        for (const value of tickPositions) {
            let pixelOffset;
            if (unit === 'm') {
                pixelOffset = value * scale;
            } else {
                pixelOffset = value * (Math.PI / 180) * scale; // Convert degrees to radians for pixel calculation
            }
            
            const x = centerX + pixelOffset;
            const y = centerY - pixelOffset; // Negative for Y because screen Y is flipped
            
            // Vertical lines (for horizontal axis values)
            if (x >= 0 && x <= canvas.width) {
                ctx.beginPath();
                ctx.moveTo(x, 0);
                ctx.lineTo(x, canvas.height);
                ctx.stroke();
                
                // Add horizontal axis labels
                const displayValue = unit === 'm' ? value.toFixed(1) : Math.round(value);
                ctx.fillText(displayValue + unit, x, centerY + 15);
            }
            
            // Horizontal lines (for vertical axis values)
            if (y >= 0 && y <= canvas.height) {
                ctx.beginPath();
                ctx.moveTo(0, y);
                ctx.lineTo(canvas.width, y);
                ctx.stroke();
                
                // Add vertical axis labels
                const displayValue = unit === 'm' ? value.toFixed(1) : Math.round(-value);
                ctx.textAlign = 'left';
                ctx.fillText(displayValue + unit, centerX + 5, y - 5);
                ctx.textAlign = 'center';
            }
        }
        
        // Reset text alignment
        ctx.textAlign = 'left';
    }
    
    drawGimbalPlot() {
        const ctx = this.gimbalCtx;
        const canvas = this.gimbalCanvas;
        const centerX = canvas.width / 2;
        const centerY = canvas.height / 2;
        
        // Clear canvas
        ctx.clearRect(0, 0, canvas.width, canvas.height);
        
        // Draw grid with units (degrees) - fixed grid, no rotation
        this.drawRotatedGridWithUnits(ctx, canvas, centerX, centerY, this.gimbalScale);
        
        // Convert telemetry data and map to proper axes (X=up, Y=yaw, Z=pitch)
        const misalignBody = { 
            x: 0,  // No X component for yaw/pitch
            y: this.misalignments.yaw,    // Yaw maps to Y axis
            z: this.misalignments.pitch   // Pitch maps to Z axis
        };
        const commandsBody = { 
            x: 0,  // No X component for yaw/pitch
            y: this.rollMixedCommands.yaw,    // Yaw maps to Y axis
            z: this.rollMixedCommands.pitch   // Pitch maps to Z axis
        };
        
        // Transform from body frame to world frame using quaternion
        const misalignWorld = this.rotateVectorByQuaternion(misalignBody, this.quaternion);
        const commandsWorld = this.rotateVectorByQuaternion(commandsBody, this.quaternion);
        
        // Convert to screen coordinates using Y and Z components (yaw=horizontal, pitch=vertical)
        const misalignX = centerX + misalignWorld.y * this.gimbalScale;  // Yaw (Y) -> horizontal
        const misalignY = centerY - misalignWorld.z * this.gimbalScale;  // Pitch (Z) -> vertical (flipped)
        const commandsX = centerX + commandsWorld.y * this.gimbalScale;  // Yaw (Y) -> horizontal
        const commandsY = centerY - commandsWorld.z * this.gimbalScale;  // Pitch (Z) -> vertical (flipped)
        
        // Draw misalignment point (red circle)
        ctx.fillStyle = '#ff3366';
        ctx.beginPath();
        ctx.arc(misalignX, misalignY, 6, 0, 2 * Math.PI);
        ctx.fill();
        
        // Draw commands point (blue circle)
        ctx.fillStyle = '#0099ff';
        ctx.beginPath();
        ctx.arc(commandsX, commandsY, 6, 0, 2 * Math.PI);
        ctx.fill();
        
        // Draw line connecting the points
        ctx.strokeStyle = '#ffffff';
        ctx.lineWidth = 2;
        ctx.beginPath();
        ctx.moveTo(misalignX, misalignY);
        ctx.lineTo(commandsX, commandsY);
        ctx.stroke();
        
        // Add axis labels (not rotated)
        ctx.fillStyle = '#aaa';
        ctx.font = '12px Arial';
        ctx.fillText('Yaw (°)', canvas.width - 45, centerY - 5);
        ctx.fillText('Pitch (°)', centerX + 5, 15);
    }
    
    drawRotatedGridWithUnits(ctx, canvas, centerX, centerY, scale) {
        ctx.strokeStyle = '#333';
        ctx.lineWidth = 1;
        
        // Draw center lines
        ctx.strokeStyle = '#555';
        ctx.lineWidth = 2;
        ctx.beginPath();
        ctx.moveTo(centerX, 0);
        ctx.lineTo(centerX, canvas.height);
        ctx.moveTo(0, centerY);
        ctx.lineTo(canvas.width, centerY);
        ctx.stroke();
        
        // For gimbal: ±5° range with 2.5° ticks
        const tickPositions = [-5, -2.5, 2.5, 5]; // Show ±5°, ±2.5°
        
        ctx.strokeStyle = '#333';
        ctx.lineWidth = 1;
        
        // Draw grid lines and labels for specific tick positions
        ctx.fillStyle = '#666';
        ctx.font = '10px Arial';
        ctx.textAlign = 'center';
        
        for (const value of tickPositions) {
            const pixelOffset = value * (Math.PI / 180) * scale; // Convert degrees to radians to pixels
            
            const x = centerX + pixelOffset;
            const y = centerY - pixelOffset; // Negative for Y because screen Y is flipped
            
            // Vertical lines (for horizontal axis values)
            if (x >= 0 && x <= canvas.width) {
                ctx.beginPath();
                ctx.moveTo(x, 0);
                ctx.lineTo(x, canvas.height);
                ctx.stroke();
                
                // Add horizontal axis labels
                ctx.fillText(value + '°', x, centerY + 15);
            }
            
            // Horizontal lines (for vertical axis values)
            if (y >= 0 && y <= canvas.height) {
                ctx.beginPath();
                ctx.moveTo(0, y);
                ctx.lineTo(canvas.width, y);
                ctx.stroke();
                
                // Add vertical axis labels
                ctx.textAlign = 'left';
                ctx.fillText((-value) + '°', centerX + 5, y - 5);
                ctx.textAlign = 'center';
            }
        }
        
        // Reset text alignment
        ctx.textAlign = 'left';
    }
    
    // Separate update functions
    updateCurrentPosition(position) {
        this.currentPosition = position;
        this.calculatePositionError();
    }
    
    updateTargetPosition(target) {
        this.targetPosition = target;
        this.calculatePositionError();
    }
    
    updateCurrentQuaternion(quaternion) {
        this.quaternion = quaternion;
    }
    
    updateAttitudeSetpoints(yawSetpoint, pitchSetpoint) {
        this.attitudeSetpoints.yaw = yawSetpoint;
        this.attitudeSetpoints.pitch = pitchSetpoint;
    }
    
    updateMisalignments(yawMisalign, pitchMisalign) {
        this.misalignments.yaw = yawMisalign;
        this.misalignments.pitch = pitchMisalign;
    }
    
    updateRollMixedCommands(rollMixedYaw, rollMixedPitch) {
        this.rollMixedCommands.yaw = rollMixedYaw;
        this.rollMixedCommands.pitch = rollMixedPitch;
    }
    
    calculatePositionError() {
        // Ignore x component, only compare y and z with setpoint
        this.positionError.y = this.currentPosition.y - this.targetPosition.y;
        this.positionError.z = this.currentPosition.z - this.targetPosition.z;
    }
    
    startAnimation() {
        const animate = () => {
            this.drawPositionPlot();
            this.drawAttitudePlot();
            this.drawGimbalPlot();
            requestAnimationFrame(animate);
        };
        animate();
    }
}

// Initialize control plots when DOM is loaded
document.addEventListener('DOMContentLoaded', () => {
    window.controlPlots = new ControlPlots();
});