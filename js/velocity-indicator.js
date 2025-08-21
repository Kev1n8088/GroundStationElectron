// Vertical Velocity Indicator
class VelocityIndicator {
    constructor() {
        this.canvas = null;
        this.ctx = null;
        this.currentVelocity = 0; // m/s
        this.minVelocity = -40;
        this.maxVelocity = 40;
        this.warningThreshold = -10; // m/s
        this.dangerThreshold = -20; // m/s
        this.initialized = false;
    }

    init() {
        if (this.initialized) return;

        this.canvas = document.getElementById('velocity-indicator');
        if (!this.canvas) {
            console.error('Velocity indicator canvas not found');
            return;
        }

        this.ctx = this.canvas.getContext('2d');
        this.initialized = true;
        this.draw();
        console.log('Velocity indicator initialized');
    }

    // Convert velocity to canvas Y position using logarithmic scale
    velocityToY(velocity) {
        const canvasHeight = this.canvas.height;
        const centerY = canvasHeight / 2;
        
        if (velocity === 0) {
            return centerY;
        }
        
        // Logarithmic scaling
        const maxLog = Math.log10(Math.abs(this.maxVelocity) + 1);
        const velocityLog = Math.log10(Math.abs(velocity) + 1);
        const normalizedLog = velocityLog / maxLog;
        
        if (velocity > 0) {
            // Positive velocity (upward)
            return centerY - (normalizedLog * centerY);
        } else {
            // Negative velocity (downward)
            return centerY + (normalizedLog * centerY);
        }
    }

    // Get color based on velocity value
    getVelocityColor(velocity) {
        if (velocity <= this.dangerThreshold) {
            return '#ff0000'; // Red - danger
        } else if (velocity <= this.warningThreshold) {
            return '#ffaa00'; // Orange - warning
        } else if (velocity < 0) {
            return '#ffff00'; // Yellow - descent
        } else if (velocity === 0) {
            return '#00ff00'; // Green - stationary
        } else {
            return '#00aaff'; // Blue - ascent
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

        // Draw center line (0 m/s)
        const centerY = height / 2;
        ctx.strokeStyle = '#ffffff';
        ctx.lineWidth = 2;
        ctx.beginPath();
        ctx.moveTo(0, centerY);
        ctx.lineTo(width, centerY);
        ctx.stroke();

        // Draw warning and danger zones
        const warningY = this.velocityToY(this.warningThreshold);
        const dangerY = this.velocityToY(this.dangerThreshold);

        // Danger zone (below -20 m/s)
        ctx.fillStyle = 'rgba(255, 0, 0, 0.2)';
        ctx.fillRect(0, dangerY, width, height - dangerY);

        // Warning zone (-20 to -10 m/s)
        ctx.fillStyle = 'rgba(255, 170, 0, 0.2)';
        ctx.fillRect(0, warningY, width, dangerY - warningY);

        // Draw scale marks
        ctx.strokeStyle = '#8b949e';
        ctx.lineWidth = 1;
        const scaleValues = [40, 20, 10, 0, -10, -20, -40];
        
        scaleValues.forEach(value => {
            const y = this.velocityToY(value);
            ctx.beginPath();
            ctx.moveTo(0, y);
            ctx.lineTo(5, y);
            ctx.stroke();
        });

        // Draw current velocity indicator
        if (this.currentVelocity !== null) {
            const indicatorY = this.velocityToY(this.currentVelocity);
            const color = this.getVelocityColor(this.currentVelocity);

            // Draw velocity bar
            const barHeight = Math.abs(indicatorY - centerY);
            if (this.currentVelocity >= 0) {
                // Upward velocity
                ctx.fillStyle = color;
                ctx.fillRect(0, indicatorY, width, barHeight);
            } else {
                // Downward velocity
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

            // Draw velocity value text
            ctx.fillStyle = '#ffffff';
            ctx.font = 'bold 16px "Futura Md BT"';
            ctx.textAlign = 'center';
            const velocityText = `${this.currentVelocity.toFixed(1)}`;
            
            // Position text to avoid overlap with indicator
            let textY = indicatorY - 12;
            if (textY < 20) textY = indicatorY + 25;
            
            ctx.fillText(velocityText, width / 2, textY);
        }
    }

    updateVelocity(velocity) {
        this.currentVelocity = velocity;
        this.draw();
    }

    resize() {
        if (this.initialized) {
            this.draw();
        }
    }
}

// Global instance
const velocityIndicator = new VelocityIndicator();

// Initialize velocity indicator when DOM is loaded
document.addEventListener('DOMContentLoaded', () => {
    setTimeout(() => {
        velocityIndicator.init();
    }, 100);
});

// Export for use in other modules
if (typeof module !== 'undefined' && module.exports) {
    module.exports = { VelocityIndicator, velocityIndicator };
}
