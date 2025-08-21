// Roll Rate Indicator
class RollRateIndicator {
    constructor() {
        this.canvas = null;
        this.ctx = null;
        this.currentRollRate = 0; // rad/s
        this.minRollRate = -5;
        this.maxRollRate = 5;
        this.warningThreshold = 1; // rad/s (absolute value)
        this.dangerThreshold = 2; // rad/s (absolute value)
        this.initialized = false;
    }

    init() {
        if (this.initialized) return;

        this.canvas = document.getElementById('roll-rate-indicator');
        if (!this.canvas) {
            console.error('Roll rate indicator canvas not found');
            return;
        }

        this.ctx = this.canvas.getContext('2d');
        this.initialized = true;
        this.draw();
        console.log('Roll rate indicator initialized');
    }

    // Convert roll rate to canvas Y position using logarithmic scale
    rollRateToY(rollRate) {
        const canvasHeight = this.canvas.height;
        const centerY = canvasHeight / 2;
        
        if (rollRate === 0) {
            return centerY;
        }
        
        // Logarithmic scaling
        const maxLog = Math.log10(Math.abs(this.maxRollRate) + 1);
        const rollRateLog = Math.log10(Math.abs(rollRate) + 1);
        const normalizedLog = rollRateLog / maxLog;
        
        if (rollRate > 0) {
            // Positive roll rate (upward)
            return centerY - (normalizedLog * centerY);
        } else {
            // Negative roll rate (downward)
            return centerY + (normalizedLog * centerY);
        }
    }

    // Get color based on roll rate value (using absolute value for thresholds)
    getRollRateColor(rollRate) {
        const absRollRate = Math.abs(rollRate);
        
        if (absRollRate >= this.dangerThreshold) {
            return '#ff0000'; // Red - danger
        } else if (absRollRate >= this.warningThreshold) {
            return '#ffaa00'; // Orange - warning
        } else if (rollRate > 0) {
            return '#00aaff'; // Blue - positive roll
        } else if (rollRate < 0) {
            return '#aa00ff'; // Purple - negative roll
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
        const warningYPos = this.rollRateToY(this.warningThreshold);
        const warningYNeg = this.rollRateToY(-this.warningThreshold);
        const dangerYPos = this.rollRateToY(this.dangerThreshold);
        const dangerYNeg = this.rollRateToY(-this.dangerThreshold);

        // Danger zones (above +2 and below -2 rad/s)
        ctx.fillStyle = 'rgba(255, 0, 0, 0.2)';
        ctx.fillRect(0, 0, width, dangerYPos); // Top danger zone
        ctx.fillRect(0, dangerYNeg, width, height - dangerYNeg); // Bottom danger zone

        // Warning zones (1 to 2 rad/s absolute)
        ctx.fillStyle = 'rgba(255, 170, 0, 0.2)';
        ctx.fillRect(0, dangerYPos, width, warningYPos - dangerYPos); // Top warning zone
        ctx.fillRect(0, warningYNeg, width, dangerYNeg - warningYNeg); // Bottom warning zone

        // Draw scale marks
        ctx.strokeStyle = '#8b949e';
        ctx.lineWidth = 1;
        const scaleValues = [5, 2, 1, 0, -1, -2, -5];
        
        scaleValues.forEach(value => {
            const y = this.rollRateToY(value);
            ctx.beginPath();
            ctx.moveTo(0, y);
            ctx.lineTo(5, y);
            ctx.stroke();
        });

        // Draw current roll rate indicator
        if (this.currentRollRate !== null) {
            const indicatorY = this.rollRateToY(this.currentRollRate);
            const color = this.getRollRateColor(this.currentRollRate);

            // Draw roll rate bar
            const barHeight = Math.abs(indicatorY - centerY);
            if (this.currentRollRate >= 0) {
                // Positive roll rate
                ctx.fillStyle = color;
                ctx.fillRect(0, indicatorY, width, barHeight);
            } else {
                // Negative roll rate
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

            // Draw roll rate value text
            ctx.fillStyle = '#ffffff';
            ctx.font = 'bold 16px "Futura Md BT"';
            ctx.textAlign = 'center';
            const rollRateText = `${this.currentRollRate.toFixed(1)}`;
            
            // Position text to avoid overlap with indicator
            let textY = indicatorY - 12;
            if (textY < 20) textY = indicatorY + 25;
            
            ctx.fillText(rollRateText, width / 2, textY);
        }
    }

    updateRollRate(rollRate) {
        this.currentRollRate = rollRate;
        this.draw();
    }

    resize() {
        if (this.initialized) {
            this.draw();
        }
    }
}

// Global instance
const rollRateIndicator = new RollRateIndicator();

// Initialize roll rate indicator when DOM is loaded
document.addEventListener('DOMContentLoaded', () => {
    setTimeout(() => {
        rollRateIndicator.init();
    }, 100);
});

// Export for use in other modules
if (typeof module !== 'undefined' && module.exports) {
    module.exports = { RollRateIndicator, rollRateIndicator };
}
