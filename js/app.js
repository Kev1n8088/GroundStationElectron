// Main application controller
class GroundStationApp {
    constructor() {
        this.initialized = false;
        this.animationId = null;
    }

    async init() {
        try {
            console.log('Initializing Ground Station GUI...');
            
            // Initialize Three.js scene
            if (!threeScene.init()) {
                throw new Error('Failed to initialize 3D scene');
            }
            console.log('✓ 3D Scene initialized');
            
            // Initialize charts
            chartManager.init();
            console.log('✓ Charts initialized');
            
            // Try to auto-connect to serial port
            try {
                await serialComm.autoConnect();
                console.log('✓ Serial auto-connected');
            } catch (error) {
                console.log('Serial auto-connect failed, manual connection required');
            }
            
            // Setup keyboard shortcuts
            this.setupKeyboardShortcuts();
            console.log('✓ Keyboard shortcuts registered');
            
            // Start animation loop
            this.startAnimationLoop();
            console.log('✓ Animation loop started');
            
            // Setup UI event handlers
            this.setupUIHandlers();
            console.log('✓ UI handlers registered');
            
            this.initialized = true;
            console.log('Ground Station GUI initialized successfully');
            
        } catch (error) {
            console.error('Failed to initialize application:', error);
            this.showError('Failed to initialize application: ' + error.message);
        }
    }

    setupKeyboardShortcuts() {
        document.addEventListener('keydown', (event) => {
            // Don't process shortcuts if typing in input fields
            if (event.target.tagName === 'INPUT') return;
            
            switch (event.code) {
                case 'Space':
                    event.preventDefault();
                    toggleSimulation();
                    break;
                    
                case 'KeyC':
                    if (event.ctrlKey) {
                        event.preventDefault();
                        connectSerial();
                    }
                    break;
                    
                case 'KeyH':
                    if (event.ctrlKey) {
                        event.preventDefault();
                        returnToHome();
                    }
                    break;
                    
                case 'KeyL':
                    if (event.ctrlKey) {
                        event.preventDefault();
                        loadObjModel();
                    }
                    break;
                    
                case 'KeyR':
                    if (event.ctrlKey) {
                        event.preventDefault();
                        this.resetView();
                    }
                    break;
                    
                case 'KeyA':
                    if (event.ctrlKey) {
                        event.preventDefault();
                        sendCommand('ARM');
                    }
                    break;
                    
                case 'KeyD':
                    if (event.ctrlKey) {
                        event.preventDefault();
                        sendCommand('DISARM');
                    }
                    break;
                    
                case 'Escape':
                    simulation.stop();
                    break;
            }
        });
    }

    setupUIHandlers() {
        // Add tooltips to buttons
        const buttons = document.querySelectorAll('button');
        buttons.forEach(button => {
            switch (button.textContent) {
                case 'Connect Serial':
                    button.title = 'Connect to serial port (Ctrl+C)';
                    break;
                case 'Load 3D Model':
                    button.title = 'Load OBJ 3D model (Ctrl+L)';
                    break;
                case 'ARM Vehicle':
                    button.title = 'ARM the vehicle (Ctrl+A)';
                    break;
                case 'DISARM Vehicle':
                    button.title = 'DISARM the vehicle (Ctrl+D)';
                    break;
                case 'Return Home':
                    button.title = 'Return to home position (Ctrl+H)';
                    break;
                case 'Toggle Simulation':
                    button.title = 'Start/stop simulation (Space)';
                    break;
                case 'Set Position':
                    button.title = 'Manually set vehicle position';
                    break;
                case 'Update Scale':
                    button.title = 'Update 3D model scale';
                    break;
            }
        });

        // Add input validation
        const numberInputs = document.querySelectorAll('input[type="number"]');
        numberInputs.forEach(input => {
            input.addEventListener('input', (event) => {
                const value = parseFloat(event.target.value);
                if (isNaN(value)) {
                    event.target.style.borderColor = '#ff4444';
                } else {
                    event.target.style.borderColor = '#30363d';
                }
            });
        });

        // Handle Enter key in manual position inputs
        const positionInputs = ['manual-x', 'manual-y', 'manual-z'];
        positionInputs.forEach(id => {
            const input = document.getElementById(id);
            if (input) {
                input.addEventListener('keydown', (event) => {
                    if (event.key === 'Enter') {
                        setPosition();
                    }
                });
            }
        });

        // Handle Enter key in scale input
        const scaleInput = document.getElementById('model-scale');
        if (scaleInput) {
            scaleInput.addEventListener('keydown', (event) => {
                if (event.key === 'Enter') {
                    updateModelScale();
                }
            });
        }
    }

    startAnimationLoop() {
        const animate = () => {
            this.animationId = requestAnimationFrame(animate);
            
            // Render the 3D scene
            threeScene.render();
            
            // Update any other continuous processes here
        };
        
        animate();
    }

    stopAnimationLoop() {
        if (this.animationId) {
            cancelAnimationFrame(this.animationId);
            this.animationId = null;
        }
    }

    resetView() {
        // Reset camera position
        if (threeScene.initialized) {
            camera.position.set(15, 12, 15);
            camera.lookAt(0, 0, 0);
        }
        
        // Clear charts
        clearCharts();
        
        // Reset vehicle position
        vehicleState.position = { x: 0, y: 5, z: 0 };
        vehicleState.euler = { roll: 0, pitch: 0, yaw: 0 };
        
        // Update displays
        if (simulation.isRunning) {
            simulation.updateDisplays();
        }
        
        console.log('View reset to defaults');
    }

    showError(message) {
        // Create error modal/notification
        const errorDiv = document.createElement('div');
        errorDiv.style.cssText = `
            position: fixed;
            top: 50%;
            left: 50%;
            transform: translate(-50%, -50%);
            background: #ff4444;
            color: white;
            padding: 20px;
            border-radius: 8px;
            z-index: 1000;
            max-width: 400px;
            text-align: center;
            box-shadow: 0 4px 8px rgba(0,0,0,0.5);
        `;
        errorDiv.innerHTML = `
            <h3>Error</h3>
            <p>${message}</p>
            <button onclick="this.parentElement.remove()" 
                    style="margin-top: 10px; padding: 8px 16px; background: white; color: #ff4444; border: none; border-radius: 4px; cursor: pointer;">
                OK
            </button>
        `;
        document.body.appendChild(errorDiv);
        
        // Auto-remove after 10 seconds
        setTimeout(() => {
            if (errorDiv.parentElement) {
                errorDiv.remove();
            }
        }, 10000);
    }

    showStatus(message, type = 'info') {
        console.log(`[${type.toUpperCase()}] ${message}`);
        
        // Could add toast notifications here
        const statusDiv = document.createElement('div');
        statusDiv.style.cssText = `
            position: fixed;
            top: 20px;
            right: 20px;
            background: ${type === 'error' ? '#ff4444' : type === 'success' ? '#44ff44' : '#4444ff'};
            color: white;
            padding: 12px 20px;
            border-radius: 6px;
            z-index: 1000;
            max-width: 300px;
            font-size: 12px;
            opacity: 0.9;
            transition: opacity 0.3s;
        `;
        statusDiv.textContent = message;
        document.body.appendChild(statusDiv);
        
        // Auto-remove after 3 seconds
        setTimeout(() => {
            statusDiv.style.opacity = '0';
            setTimeout(() => {
                if (statusDiv.parentElement) {
                    statusDiv.remove();
                }
            }, 300);
        }, 3000);
    }

    // Application lifecycle methods
    onBeforeUnload() {
        // Cleanup before app closes
        this.stopAnimationLoop();
        simulation.stop();
        if (serialComm.isConnected) {
            serialComm.disconnect();
        }
        console.log('Application cleanup completed');
    }

    // Export telemetry data
    exportTelemetryData() {
        const data = {
            timestamp: new Date().toISOString(),
            vehicleState: { ...vehicleState },
            simulationStatus: simulation.getStatus(),
            chartData: chartManager.exportChartData()
        };
        
        const blob = new Blob([JSON.stringify(data, null, 2)], { type: 'application/json' });
        const url = URL.createObjectURL(blob);
        
        const a = document.createElement('a');
        a.href = url;
        a.download = `telemetry_${new Date().toISOString().slice(0, 19).replace(/:/g, '-')}.json`;
        document.body.appendChild(a);
        a.click();
        document.body.removeChild(a);
        URL.revokeObjectURL(url);
        
        this.showStatus('Telemetry data exported', 'success');
    }
}

// Global app instance
const app = new GroundStationApp();

// Initialize when DOM is loaded
document.addEventListener('DOMContentLoaded', () => {
    app.init().catch(error => {
        console.error('Application initialization failed:', error);
    });
});

// Handle app shutdown
window.addEventListener('beforeunload', () => {
    app.onBeforeUnload();
});

// Global utility functions
function resetView() {
    app.resetView();
}

function exportTelemetry() {
    app.exportTelemetryData();
}

// Development helpers (remove in production)
if (process.argv && process.argv.includes('--dev')) {
    window.app = app;
    window.threeScene = threeScene;
    window.simulation = simulation;
    window.serialComm = serialComm;
    window.chartManager = chartManager;
    window.modelLoader = modelLoader;
    
    console.log('Development mode: Global objects exposed to window');
}