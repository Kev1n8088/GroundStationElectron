class KalmanStdIndicator {
    constructor() {
        this.canvas = document.getElementById('kalman-std-chart');
        this.ctx = this.canvas.getContext('2d');
        
        // Data storage for standard deviations (in appropriate units)
        this.stdDevs = {
            position: { x: 0.01, y: 0.01, z: 0.01 },    // meters
            velocity: { x: 0.01, y: 0.01, z: 0.01 },    // m/s
            acceleration: { x: 0.01, y: 0.01, z: 0.01 } // m/s²
        };
        
        // Color scheme for X, Y, Z axes (matching existing design)
        this.colors = {
            x: '#44ff44',  // Green 
            y: '#ff3366',  // Red 
            z: '#0099ff'   // Blue
        };
        
        // Logarithmic scale parameters
        this.minValue = 0.001;  // Minimum displayable value
        this.maxValue = 10.0;   // Maximum displayable value
        
        // Thresholds for color coding (good/caution/bad)
        this.thresholds = {
            position: { good: 0.05, caution: 0.1 },     // meters
            velocity: { good: 0.1, caution: 0.3 },     // m/s
            acceleration: { good: 0.2, caution: 1.0 }  // m/s²
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
        const containerWidth = container.clientWidth;
        const containerHeight = container.clientHeight;
        
        // Set both CSS size and canvas resolution to the same values to prevent stretching
        this.canvas.style.width = containerWidth + 'px';
        this.canvas.style.height = containerHeight + 'px';
        this.canvas.width = containerWidth;
        this.canvas.height = containerHeight;
    }
    
    // Convert value to logarithmic scale (0 to 1)
    valueToLogScale(value) {
        if (value <= this.minValue) return 0;
        if (value >= this.maxValue) return 1;
        
        const minLog = Math.log10(this.minValue);
        const maxLog = Math.log10(this.maxValue);
        const valueLog = Math.log10(value);
        
        return (valueLog - minLog) / (maxLog - minLog);
    }
    
    // Convert log scale back to value for labels
    logScaleToValue(scale) {
        const minLog = Math.log10(this.minValue);
        const maxLog = Math.log10(this.maxValue);
        const valueLog = minLog + scale * (maxLog - minLog);
        return Math.pow(10, valueLog);
    }
    
    drawChart() {
        const ctx = this.ctx;
        const canvas = this.canvas;
        
        // Clear canvas with background color
        ctx.fillStyle = '#0d1117';
        ctx.fillRect(0, 0, canvas.width, canvas.height);
        
        // Chart dimensions
        const margin = { top: 15, right: 35, bottom: 55, left: 45 };
        const chartWidth = canvas.width - margin.left - margin.right;
        const chartHeight = canvas.height - margin.top - margin.bottom;
        
        // Draw border
        ctx.strokeStyle = '#30363d';
        ctx.lineWidth = 1;
        ctx.strokeRect(margin.left, margin.top, chartWidth, chartHeight);
        
        // Group configuration
        const groups = ['Pos', 'Vel', 'Acc'];  // Shortened labels for space
        const groupWidth = chartWidth / groups.length;
        const barWidth = (groupWidth - 8) / 4; // 3 bars + spacing, tighter
        
        // Draw background grid
        this.drawLogGrid(ctx, margin, chartWidth, chartHeight);
        
        // Draw bars for each group
        groups.forEach((group, groupIndex) => {
            const fullGroupName = ['position', 'velocity', 'acceleration'][groupIndex];
            const groupLabels = ['Pos (m)', 'Vel (m/s)', 'Acc (m/s²)'];
            const groupX = margin.left + groupIndex * groupWidth + 4;
            const data = this.stdDevs[fullGroupName];
            
            // Draw group label with units (further lowered position)
            ctx.fillStyle = '#8b949e';
            ctx.font = '15px "Futura Md BT"';
            ctx.textAlign = 'center';
            ctx.fillText(groupLabels[groupIndex], groupX - 12 + groupWidth/2, canvas.height);
            // Draw bars for X, Y, Z
            ['x', 'y', 'z'].forEach((axis, axisIndex) => {
                const barX = groupX + axisIndex * barWidth + barWidth/4;
                const value = Math.max(data[axis], this.minValue);
                const logScale = this.valueToLogScale(value);
                const barHeight = logScale * chartHeight;
                const barY = margin.top + chartHeight - barHeight;
                
                // Determine bar opacity based on thresholds
                const thresholds = this.thresholds[fullGroupName];
                const opacity = this.getBarOpacity(value, thresholds);
                
                // Draw bar with background
                ctx.fillStyle = '#161b22'; // Dark background
                ctx.fillRect(barX, margin.top, barWidth - 1, chartHeight);
                
                // Draw colored bar
                const baseColor = this.colors[axis];
                ctx.fillStyle = this.getColorWithOpacity(baseColor, opacity);
                ctx.fillRect(barX, barY, barWidth - 1, barHeight);
                
                // Draw bar border
                ctx.strokeStyle = '#30363d';
                ctx.lineWidth = 1;
                ctx.strokeRect(barX, margin.top, barWidth - 1, chartHeight);
                
                // Draw value text ABOVE the bar for better visibility
                ctx.fillStyle = '#ffffff';
                ctx.font = '12px "Futura Md BT"';
                ctx.textAlign = 'center';
                const displayValue = value < 0.01 ? value.toExponential(0) : value.toFixed(2);
                ctx.fillText(displayValue, barX + barWidth/2, barY - 4);
                
                // Draw axis label at bottom of each bar (further adjusted position)
                ctx.fillStyle = '#8b949e';
                ctx.font = '13px "Futura Md BT"';
                ctx.textAlign = 'center';
                ctx.fillText(axis.toUpperCase(), barX + barWidth/2, canvas.height - 28);
            });
        });
        
        // Note: Y-axis label is now handled by the vertical text in HTML/CSS following velocity indicator pattern
    }
    
    drawLogGrid(ctx, margin, chartWidth, chartHeight) {
        // Draw logarithmic grid lines and labels
        const gridScales = [0, 0.2, 0.4, 0.6, 0.8, 1.0];
        
        ctx.strokeStyle = '#21262d';
        ctx.lineWidth = 1;
        
        gridScales.forEach(scale => {
            const y = margin.top + chartHeight - (scale * chartHeight);
            
            // Draw grid line
            ctx.beginPath();
            ctx.moveTo(margin.left, y);
            ctx.lineTo(margin.left + chartWidth, y);
            ctx.stroke();
            
            // Draw Y-axis labels
            const value = this.logScaleToValue(scale);
            ctx.fillStyle = '#8b949e';
            ctx.font = '12px "Futura Md BT"';
            ctx.textAlign = 'right';
            
            let displayValue;
            if (value >= 1) {
                displayValue = value.toFixed(0);
            } else if (value >= 0.01) {
                displayValue = value.toFixed(2);
            } else {
                displayValue = value.toExponential(0);
            }
            
            ctx.fillText(displayValue, margin.left - 3, y + 2);
        });
        
        // Draw center reference line (0.1 threshold)
        const refScale = this.valueToLogScale(0.1);
        const refY = margin.top + chartHeight - (refScale * chartHeight);
        ctx.strokeStyle = '#ffffff';
        ctx.lineWidth = 1;
        ctx.setLineDash([3, 3]);
        ctx.beginPath();
        ctx.moveTo(margin.left, refY);
        ctx.lineTo(margin.left + chartWidth, refY);
        ctx.stroke();
        ctx.setLineDash([]);
    }
    
    getBarOpacity(value, thresholds) {
        if (value <= thresholds.good) return 1.0;      // Good - full opacity
        if (value <= thresholds.caution) return 0.7;   // Caution - dimmed
        return 0.4;                                     // Bad - very dim
    }
    
    getColorWithOpacity(hexColor, opacity) {
        // Convert hex to RGB and apply opacity
        const r = parseInt(hexColor.substr(1, 2), 16);
        const g = parseInt(hexColor.substr(3, 2), 16);
        const b = parseInt(hexColor.substr(5, 2), 16);
        
        return `rgba(${r}, ${g}, ${b}, ${opacity})`;
    }
    
    // Update functions for receiving telemetry data
    updatePositionStdDev(x, y, z) {
        this.stdDevs.position = { x, y, z };
    }
    
    updateVelocityStdDev(x, y, z) {
        this.stdDevs.velocity = { x, y, z };
    }
    
    updateAccelerationStdDev(x, y, z) {
        this.stdDevs.acceleration = { x, y, z };
    }
    
    // Batch update function
    updateAllStdDevs(position, velocity, acceleration) {
        this.stdDevs.position = position;
        this.stdDevs.velocity = velocity;
        this.stdDevs.acceleration = acceleration;
    }
    
    startAnimation() {
        const animate = () => {
            this.drawChart();
            requestAnimationFrame(animate);
        };
        animate();
    }
}

// Initialize when DOM is loaded
document.addEventListener('DOMContentLoaded', () => {
    window.kalmanStdIndicator = new KalmanStdIndicator();
});
