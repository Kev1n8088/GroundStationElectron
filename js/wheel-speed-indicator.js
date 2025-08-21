// Wheel Speed Indicator
class WheelSpeedIndicator {
    constructor() {
        this.canvas = null;
        this.ctx = null;
        this.currentWheelSpeed = 0; // rad/s
        this.minWheelSpeed = -1600;
        this.maxWheelSpeed = 1600;
        this.warningThreshold = 800; // rad/s (absolute value)
        this.dangerThreshold = 1200; // rad/s (absolute value)
        this.initialized = false;
    }

    init() {
        if (this.initialized) return;

        this.canvas = document.getElementById('wheel-speed-indicator');
        if (!this.canvas) {
            console.error('Wheel speed indicator canvas not found');
            return;
        }

        this.ctx = this.canvas.getContext('2d');
        this.initialized = true;
        this.draw();
        console.log('Wheel speed indicator initialized');
    }

    // Convert wheel speed to canvas Y position using linear scale
    wheelSpeedToY(wheelSpeed) {
        const canvasHeight = this.canvas.height;
        const centerY = canvasHeight / 2;
        
        // Linear scaling from -1600 to +1600 rad/s
        const normalizedSpeed = wheelSpeed / this.maxWheelSpeed;
        return centerY - (normalizedSpeed * centerY);
    }

    // Get color based on wheel speed value (using absolute value for thresholds)
    getWheelSpeedColor(wheelSpeed) {
        const absWheelSpeed = Math.abs(wheelSpeed);
        
        if (absWheelSpeed >= this.dangerThreshold) {
            return '#ff0000'; // Red - danger
        } else if (absWheelSpeed >= this.warningThreshold) {
            return '#ffaa00'; // Orange - warning
        } else if (wheelSpeed > 0) {
            return '#00aaff'; // Blue - positive wheel speed
        } else if (wheelSpeed < 0) {
            return '#aa00ff'; // Purple - negative wheel speed
        } else {
            return '#00ff00'; // Green - stationary
        }
    }

    draw() {
        if (!this.initialized || !this.ctx) return;

        const ctx = this.ctx;
        const width = this.canvas.width;
        const height = this.canvas.height;

        // Clear canvas
        ctx.clearRect(0, 0, width, height);

        // Draw background bar
        ctx.fillStyle = '#0d1117';
        ctx.fillRect(0, 0, width, height);

        // Draw border
        ctx.strokeStyle = '#30363d';
        ctx.lineWidth = 1;
        ctx.strokeRect(0, 0, width, height);

        // Draw center line (0 rad/s)
        const centerY = height / 2;
        ctx.strokeStyle = '#ffffff';
        ctx.lineWidth = 2;
        ctx.beginPath();
        ctx.moveTo(0, centerY);
        ctx.lineTo(width, centerY);
        ctx.stroke();

        // Draw warning and danger zones (symmetric around center)
        const warningYPos = this.wheelSpeedToY(this.warningThreshold);
        const warningYNeg = this.wheelSpeedToY(-this.warningThreshold);
        const dangerYPos = this.wheelSpeedToY(this.dangerThreshold);
        const dangerYNeg = this.wheelSpeedToY(-this.dangerThreshold);

        // Danger zones (above +1200 and below -1200 rad/s)
        ctx.fillStyle = 'rgba(255, 0, 0, 0.2)';
        ctx.fillRect(0, 0, width, dangerYPos); // Top danger zone
        ctx.fillRect(0, dangerYNeg, width, height - dangerYNeg); // Bottom danger zone

        // Warning zones (800 to 1200 rad/s absolute)
        ctx.fillStyle = 'rgba(255, 170, 0, 0.2)';
        ctx.fillRect(0, dangerYPos, width, warningYPos - dangerYPos); // Top warning zone
        ctx.fillRect(0, warningYNeg, width, dangerYNeg - warningYNeg); // Bottom warning zone

        // Draw scale marks
        ctx.strokeStyle = '#8b949e';
        ctx.lineWidth = 1;
        const scaleValues = [1600, 1200, 800, 400, 0, -400, -800, -1200, -1600];
        
        scaleValues.forEach(value => {
            const y = this.wheelSpeedToY(value);
            ctx.beginPath();
            ctx.moveTo(0, y);
            ctx.lineTo(5, y);
            ctx.stroke();
        });

        // Draw current wheel speed indicator
        if (this.currentWheelSpeed !== null) {
            const indicatorY = this.wheelSpeedToY(this.currentWheelSpeed);
            const color = this.getWheelSpeedColor(this.currentWheelSpeed);

            // Draw wheel speed bar
            const barHeight = Math.abs(indicatorY - centerY);
            if (this.currentWheelSpeed >= 0) {
                // Positive wheel speed
                ctx.fillStyle = color;
                ctx.fillRect(0, indicatorY, width, barHeight);
            } else {
                // Negative wheel speed
                ctx.fillStyle = color;
                ctx.fillRect(0, centerY, width, barHeight);
            }

            // Draw current value indicator line
            ctx.strokeStyle = '#ffffff';
            ctx.lineWidth = 3;
            ctx.beginPath();
            ctx.moveTo(0, indicatorY);
            ctx.lineTo(width, indicatorY);
            ctx.stroke();

            // Draw wheel speed value text
            ctx.fillStyle = '#ffffff';
            ctx.font = 'bold 16px "Futura Md BT"';
            ctx.textAlign = 'center';
            const wheelSpeedText = `${Math.round(this.currentWheelSpeed)}`;
            
            // Position text to avoid overlap with indicator
            let textY = indicatorY - 12;
            if (textY < 20) textY = indicatorY + 25;
            
            ctx.fillText(wheelSpeedText, width / 2, textY);
        }
    }

    updateWheelSpeed(wheelSpeed) {
        this.currentWheelSpeed = wheelSpeed;
        this.draw();
    }

    resize() {
        if (this.initialized) {
            this.draw();
        }
    }
}

// Global instance
const wheelSpeedIndicator = new WheelSpeedIndicator();

// Initialize wheel speed indicator when DOM is loaded
document.addEventListener('DOMContentLoaded', () => {
    setTimeout(() => {
        wheelSpeedIndicator.init();
    }, 100);
});

// Export for use in other modules
if (typeof module !== 'undefined' && module.exports) {
    module.exports = { WheelSpeedIndicator, wheelSpeedIndicator };
}
