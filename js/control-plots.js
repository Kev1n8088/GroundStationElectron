class ControlPlots {
    constructor() {
        this.positionCanvas = document.getElementById('position-plot');
        this.attitudeCanvas = document.getElementById('attitude-plot');
        
        this.positionCtx = this.positionCanvas.getContext('2d');
        this.attitudeCtx = this.attitudeCanvas.getContext('2d');
        
        // Plot parameters - adjusted for limited ranges
        this.positionScale = 100; // pixels per meter (±1m range)
        this.attitudeScale = 200; // pixels per radian (±30° range)
        
        // Data storage
        this.currentPosition = { x: 0, y: 0, z: 0 };
        this.targetPosition = { y: 0, z: 0 };
        this.positionError = { y: 0, z: 0 };
        this.quaternion = { w: 1, x: 0, y: 0, z: 0 };
        this.attitudeSetpoints = { yaw: 0, pitch: 0 };
        
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
        
        // Set both canvases to be square
        [this.positionCanvas, this.attitudeCanvas].forEach(canvas => {
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
    
    calculatePositionError() {
        // Ignore x component, only compare y and z with setpoint
        this.positionError.y = this.currentPosition.y - this.targetPosition.y;
        this.positionError.z = this.currentPosition.z - this.targetPosition.z;
    }
    
    startAnimation() {
        const animate = () => {
            this.drawPositionPlot();
            this.drawAttitudePlot();
            requestAnimationFrame(animate);
        };
        animate();
    }
}

// Initialize control plots when DOM is loaded
document.addEventListener('DOMContentLoaded', () => {
    window.controlPlots = new ControlPlots();
});