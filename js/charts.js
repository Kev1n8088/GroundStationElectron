// Flight Data Chart - Thrust and Altitude vs Time Since Launch
class FlightDataChart {
    constructor() {
        this.canvas = document.getElementById('flight-data-chart');
        if (!this.canvas) {
            console.error('Flight data chart canvas element not found!');
            return;
        }
        this.ctx = this.canvas.getContext('2d');
        
        // Chart parameters
        this.maxTime = 15.0; // 15 seconds maximum
        this.maxValue = 25.0; // 25N thrust, 25m altitude
        this.timeStep = 0.1; // Store data every 100ms
        
        // Data storage
        this.timeData = [];
        this.thrustData = [];
        this.altitudeData = [];
        this.stateTransitions = []; // Array of {time, fromState, toState}
        
        // Current values
        this.currentThrust = 0;
        this.currentAltitude = 0;
        this.currentTime = 0;
        this.lastVehicleState = null;
        
        // Chart styling (matching control-plots.js)
        this.colors = {
            thrust: '#ff3366',      // Red for thrust
            altitude: '#0099ff',    // Blue for altitude
            grid: '#333',
            centerGrid: '#555',
            text: '#aaa',
            background: '#161b22',
            stateTransition: '#ffb366' // Orange for state transitions
        };
        
        this.initializeCanvas();
        this.startAnimation();
    }
    
    initializeCanvas() {
        this.resizeCanvas();
        window.addEventListener('resize', () => this.resizeCanvas());
    }
    
    resizeCanvas() {
        const container = this.canvas.parentElement;
        if (!container) {
            console.warn('Flight chart container not found');
            return;
        }
        
        const containerWidth = container.clientWidth;
        const containerHeight = container.clientHeight;
        
        // Prevent zero-size canvas which could cause blank overlay issues
        if (containerWidth <= 0 || containerHeight <= 0) {
            console.warn('Flight chart container has zero dimensions:', containerWidth, containerHeight);
            return;
        }
        
        // Only resize if dimensions actually changed to prevent unnecessary redraws
        if (this.canvas.width === containerWidth && this.canvas.height === containerHeight) {
            return;
        }
        
        // Set both CSS size and canvas resolution to prevent stretching
        this.canvas.style.width = containerWidth + 'px';
        this.canvas.style.height = containerHeight + 'px';
        this.canvas.width = containerWidth;
        this.canvas.height = containerHeight;
        
        console.log('Flight chart resized to:', containerWidth, 'x', containerHeight);
        
        // Force a redraw after resize
        setTimeout(() => {
            this.drawChart();
        }, 10);
    }
    
    // Vehicle state enum mapping (from serial-comm.js)
    getVehicleStateName(state) {
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
        return VEHICLE_STATES[state] || `STATE_${state}`;
    }
    
    updateData(thrust, altitude, timeSinceLaunch, vehicleState) {
        // Don't plot if time since launch is 0
        if (timeSinceLaunch <= 0) {
            return;
        }
        
        // Don't add data beyond max time
        if (timeSinceLaunch > this.maxTime) {
            return;
        }
        
        this.currentThrust = thrust;
        this.currentAltitude = altitude;
        this.currentTime = timeSinceLaunch;
        
        // Check for vehicle state transition
        if (this.lastVehicleState !== null && this.lastVehicleState !== vehicleState) {
            this.stateTransitions.push({
                time: timeSinceLaunch,
                fromState: this.getVehicleStateName(this.lastVehicleState),
                toState: this.getVehicleStateName(vehicleState)
            });
        }
        this.lastVehicleState = vehicleState;
        
        // Add data point (only if we have some data already or this is first non-zero point)
        if (this.timeData.length === 0 || 
            timeSinceLaunch >= this.timeData[this.timeData.length - 1] + this.timeStep) {
            
            this.timeData.push(timeSinceLaunch);
            this.thrustData.push(thrust);
            this.altitudeData.push(altitude);
            
            // Remove old data points beyond max time
            while (this.timeData.length > 0 && this.timeData[0] < timeSinceLaunch - this.maxTime) {
                this.timeData.shift();
                this.thrustData.shift();
                this.altitudeData.shift();
            }
            
            // Clean up old state transitions
            this.stateTransitions = this.stateTransitions.filter(
                transition => transition.time >= timeSinceLaunch - this.maxTime
            );
        }
    }
    
    drawChart() {
        if (!this.ctx || !this.canvas) {
            return;
        }
        
        const ctx = this.ctx;
        const canvas = this.canvas;
        
        // Clear canvas
        ctx.fillStyle = this.colors.background;
        ctx.fillRect(0, 0, canvas.width, canvas.height);
        
        // Chart dimensions with reduced margins for more chart area
        const margin = { top: 20, right: 90, bottom: 40, left: 60 };
        const chartWidth = canvas.width - margin.left - margin.right;
        const chartHeight = canvas.height - margin.top - margin.bottom;
        
        if (chartWidth <= 0 || chartHeight <= 0) return;
        
        // Always draw border and background for chart area
        ctx.strokeStyle = '#30363d';
        ctx.lineWidth = 1;
        ctx.strokeRect(margin.left, margin.top, chartWidth, chartHeight);
        
        // Draw grid and axes (always visible with units)
        this.drawGrid(ctx, margin, chartWidth, chartHeight);
        this.drawAxes(ctx, margin, chartWidth, chartHeight);
        
        // Draw legend (always visible to show what will be plotted with units)
        this.drawLegend(ctx, canvas);
        
        // Draw state transition markers (only if there are any)
        if (this.stateTransitions.length > 0) {
            this.drawStateTransitions(ctx, margin, chartWidth, chartHeight);
        }
        
        // Draw data lines (only if data is available)
        if (this.timeData.length > 0) {
            this.drawDataLine(ctx, margin, chartWidth, chartHeight, this.timeData, this.thrustData, this.colors.thrust, 'Thrust');
            this.drawDataLine(ctx, margin, chartWidth, chartHeight, this.timeData, this.altitudeData, this.colors.altitude, 'Altitude');
        }
    }
    
    drawGrid(ctx, margin, chartWidth, chartHeight) {
        ctx.strokeStyle = this.colors.grid;
        ctx.lineWidth = 1;
        
        // Time grid lines (every 3 seconds)
        for (let t = 0; t <= this.maxTime; t += 3) {
            const x = margin.left + (t / this.maxTime) * chartWidth;
            ctx.beginPath();
            ctx.moveTo(x, margin.top);
            ctx.lineTo(x, margin.top + chartHeight);
            ctx.stroke();
        }
        
        // Value grid lines (every 5 units)
        for (let v = 0; v <= this.maxValue; v += 5) {
            const y = margin.top + chartHeight - (v / this.maxValue) * chartHeight;
            ctx.beginPath();
            ctx.moveTo(margin.left, y);
            ctx.lineTo(margin.left + chartWidth, y);
            ctx.stroke();
        }
        
        // Remove center lines since we're plotting 0-25 and 0-15 (not centered around 0)
    }
    
    drawAxes(ctx, margin, chartWidth, chartHeight) {
        ctx.fillStyle = this.colors.text;
        ctx.font = '11px "Futura Md BT"';
        ctx.textAlign = 'center';
        
        // X-axis labels (time) - always show units
        for (let t = 0; t <= this.maxTime; t += 3) {
            const x = margin.left + (t / this.maxTime) * chartWidth;
            ctx.fillText(`${t}s`, x, margin.top + chartHeight + 16);
        }
        
        // Y-axis labels (values) - always show units
        ctx.textAlign = 'right';
        for (let v = 0; v <= this.maxValue; v += 5) {
            const y = margin.top + chartHeight - (v / this.maxValue) * chartHeight;
            ctx.fillText(`${v}`, margin.left - 8, y + 3);
        }
        
        // Axis titles - always visible
        ctx.textAlign = 'center';
        ctx.font = '12px "Futura Md BT"';
        
        // X-axis title
        ctx.fillText('Time Since Launch (s)', margin.left + chartWidth / 2, canvas.height - 8);
        
        // Y-axis title (rotated) - with units
        ctx.save();
        ctx.translate(15, margin.top + chartHeight / 2);
        ctx.rotate(-Math.PI / 2);
        ctx.fillText('Thrust (N) / Altitude (m)', 0, 0);
        ctx.restore();
    }
    
    drawStateTransitions(ctx, margin, chartWidth, chartHeight) {
        this.stateTransitions.forEach(transition => {
            const x = margin.left + (transition.time / this.maxTime) * chartWidth;
            
            // Draw vertical line
            ctx.strokeStyle = this.colors.stateTransition;
            ctx.lineWidth = 2;
            ctx.setLineDash([5, 5]);
            ctx.beginPath();
            ctx.moveTo(x, margin.top);
            ctx.lineTo(x, margin.top + chartHeight);
            ctx.stroke();
            ctx.setLineDash([]);
            
            // Draw state label
            ctx.fillStyle = this.colors.stateTransition;
            ctx.font = '10px "Futura Md BT"';
            ctx.textAlign = 'left';
            ctx.save();
            ctx.translate(x + 2, margin.top + 15);
            ctx.rotate(Math.PI / 2);
            ctx.fillText(transition.toState, 0, 0);
            ctx.restore();
        });
    }
    
    drawDataLine(ctx, margin, chartWidth, chartHeight, timeData, valueData, color, label) {
        if (timeData.length < 2) return;
        
        ctx.strokeStyle = color;
        ctx.lineWidth = 2;
        ctx.beginPath();
        
        let firstPoint = true;
        for (let i = 0; i < timeData.length; i++) {
            const x = margin.left + (timeData[i] / this.maxTime) * chartWidth;
            const y = margin.top + chartHeight - (valueData[i] / this.maxValue) * chartHeight;
            
            if (firstPoint) {
                ctx.moveTo(x, y);
                firstPoint = false;
            } else {
                ctx.lineTo(x, y);
            }
        }
        ctx.stroke();
        
        // Draw data points
        ctx.fillStyle = color;
        for (let i = 0; i < timeData.length; i++) {
            const x = margin.left + (timeData[i] / this.maxTime) * chartWidth;
            const y = margin.top + chartHeight - (valueData[i] / this.maxValue) * chartHeight;
            
            ctx.beginPath();
            ctx.arc(x, y, 2, 0, 2 * Math.PI);
            ctx.fill();
        }
    }
    
    drawLegend(ctx, canvas) {
        const legendX = canvas.width - 85;
        const legendY = 25;
        
        ctx.font = '11px "Futura Md BT"';
        ctx.textAlign = 'left';
        
        // Legend background for better visibility
        ctx.fillStyle = 'rgba(22, 27, 34, 0.8)';
        ctx.fillRect(legendX - 5, legendY - 5, 80, 60);
        ctx.strokeStyle = '#30363d';
        ctx.lineWidth = 1;
        ctx.strokeRect(legendX - 5, legendY - 5, 80, 60);
        
        // Thrust legend
        ctx.fillStyle = this.colors.thrust;
        ctx.fillRect(legendX, legendY, 12, 2);
        ctx.fillText('Thrust (N)', legendX + 16, legendY + 8);
        
        // Altitude legend
        ctx.fillStyle = this.colors.altitude;
        ctx.fillRect(legendX, legendY + 16, 12, 2);
        ctx.fillText('Altitude (m)', legendX + 16, legendY + 24);
        
        // State transition legend
        ctx.strokeStyle = this.colors.stateTransition;
        ctx.lineWidth = 2;
        ctx.setLineDash([4, 4]);
        ctx.beginPath();
        ctx.moveTo(legendX, legendY + 35);
        ctx.lineTo(legendX + 12, legendY + 35);
        ctx.stroke();
        ctx.setLineDash([]);
        ctx.fillStyle = this.colors.stateTransition;
        ctx.fillText('State Change', legendX + 16, legendY + 40);
        
        // Units reminder
        ctx.fillStyle = this.colors.text;
        ctx.font = '9px "Futura Md BT"';
        ctx.fillText('Max: 25N/25m', legendX, legendY + 52);
    }
    
    // Clear all data (for new flight)
    clearData() {
        this.timeData = [];
        this.thrustData = [];
        this.altitudeData = [];
        this.stateTransitions = [];
        this.currentThrust = 0;
        this.currentAltitude = 0;
        this.currentTime = 0;
        this.lastVehicleState = null;
    }
    
    startAnimation() {
        const animate = () => {
            // Only draw if canvas and context are available
            if (this.canvas && this.ctx && this.canvas.width > 0 && this.canvas.height > 0) {
                this.drawChart();
            }
            requestAnimationFrame(animate);
        };
        animate();
    }
}

// Initialize when DOM is loaded
document.addEventListener('DOMContentLoaded', () => {
    window.flightDataChart = new FlightDataChart();
});

// Global function for updating chart (called from serial-comm.js)
function updateFlightChart(thrust, altitude, timeSinceLaunch, vehicleState) {
    if (typeof flightDataChart !== 'undefined') {
        flightDataChart.updateData(thrust, altitude, timeSinceLaunch, vehicleState);
    }
}

// Global function for clearing chart
function clearFlightChart() {
    if (typeof flightDataChart !== 'undefined') {
        flightDataChart.clearData();
    }
}