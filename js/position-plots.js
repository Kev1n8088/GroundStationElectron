// Position plotting functionality for separate window
class PositionPlots {
    constructor() {
        this.maxDataPoints = 1000; // Keep last 1000 points
        this.isPaused = false;
        
        // Data storage
        this.timeData = [];
        this.statePositionData = {
            x: [],
            y: [],
            z: []
        };
        this.kalmanPositionData = {
            x: [],
            y: [],
            z: []
        };
        
        this.charts = {};
        this.startTime = Date.now();
        
        this.initializeCharts();
        this.setupEventListeners();
    }
    
    initializeCharts() {
        const chartConfig = {
            type: 'line',
            options: {
                responsive: true,
                maintainAspectRatio: false,
                interaction: {
                    intersect: false,
                    mode: 'index'
                },
                plugins: {
                    legend: {
                        labels: {
                            color: '#ffffff'
                        }
                    }
                },
                scales: {
                    x: {
                        type: 'linear',
                        position: 'bottom',
                        title: {
                            display: true,
                            text: 'Time (s)',
                            color: '#ffffff'
                        },
                        grid: {
                            color: '#30363d'
                        },
                        ticks: {
                            color: '#8b949e'
                        }
                    },
                    y: {
                        title: {
                            display: true,
                            text: 'Position (m)',
                            color: '#ffffff'
                        },
                        grid: {
                            color: '#30363d'
                        },
                        ticks: {
                            color: '#8b949e'
                        }
                    }
                },
                animation: false,
                elements: {
                    point: {
                        radius: 0 // Hide points for better performance
                    },
                    line: {
                        tension: 0 // Straight lines
                    }
                }
            }
        };
        
        // X Position Chart
        this.charts.x = new Chart(document.getElementById('x-position-chart'), {
            ...chartConfig,
            data: {
                datasets: [
                    {
                        label: 'State Telemetry',
                        data: [],
                        borderColor: '#00ff00',
                        backgroundColor: 'rgba(0, 255, 0, 0.1)',
                        borderWidth: 2,
                        fill: false
                    },
                    {
                        label: 'Kalman Measurement',
                        data: [],
                        borderColor: '#ff6b6b',
                        backgroundColor: 'rgba(255, 107, 107, 0.1)',
                        borderWidth: 2,
                        fill: false
                    }
                ]
            }
        });
        
        // Y Position Chart
        this.charts.y = new Chart(document.getElementById('y-position-chart'), {
            ...chartConfig,
            data: {
                datasets: [
                    {
                        label: 'State Telemetry',
                        data: [],
                        borderColor: '#00ff00',
                        backgroundColor: 'rgba(0, 255, 0, 0.1)',
                        borderWidth: 2,
                        fill: false
                    },
                    {
                        label: 'Kalman Measurement',
                        data: [],
                        borderColor: '#ff6b6b',
                        backgroundColor: 'rgba(255, 107, 107, 0.1)',
                        borderWidth: 2,
                        fill: false
                    }
                ]
            }
        });
        
        // Z Position Chart
        this.charts.z = new Chart(document.getElementById('z-position-chart'), {
            ...chartConfig,
            data: {
                datasets: [
                    {
                        label: 'State Telemetry',
                        data: [],
                        borderColor: '#00ff00',
                        backgroundColor: 'rgba(0, 255, 0, 0.1)',
                        borderWidth: 2,
                        fill: false
                    },
                    {
                        label: 'Kalman Measurement',
                        data: [],
                        borderColor: '#ff6b6b',
                        backgroundColor: 'rgba(255, 107, 107, 0.1)',
                        borderWidth: 2,
                        fill: false
                    }
                ]
            }
        });
    }
    
    setupEventListeners() {
        // Listen for pause checkbox
        const pauseCheckbox = document.getElementById('pause-checkbox');
        pauseCheckbox.addEventListener('change', (e) => {
            this.isPaused = e.target.checked;
        });
        
        // Listen for data from main window via electron IPC
        if (typeof window !== 'undefined' && window.require) {
            const { ipcRenderer } = window.require('electron');
            
            ipcRenderer.on('state-telemetry-data', (event, data) => {
                if (!this.isPaused) {
                    this.updateStatePosition(data.position, data.timestamp);
                }
            });
            
            ipcRenderer.on('kalman-data', (event, data) => {
                if (!this.isPaused) {
                    this.updateKalmanPosition(data.measurements.position, data.timestamp);
                }
            });
        }
    }
    
    updateStatePosition(position, timestamp) {
        const timeInSeconds = (timestamp - this.startTime) / 1000;
        
        // Add data points
        this.addDataPoint(this.statePositionData.x, timeInSeconds, position.x);
        this.addDataPoint(this.statePositionData.y, timeInSeconds, position.y);
        this.addDataPoint(this.statePositionData.z, timeInSeconds, position.z);
        
        // Update charts
        this.updateChart('x', this.statePositionData.x, 0);
        this.updateChart('y', this.statePositionData.y, 0);
        this.updateChart('z', this.statePositionData.z, 0);
    }
    
    updateKalmanPosition(position, timestamp) {
        const timeInSeconds = (timestamp - this.startTime) / 1000;
        
        // Add data points
        this.addDataPoint(this.kalmanPositionData.x, timeInSeconds, position.x);
        this.addDataPoint(this.kalmanPositionData.y, timeInSeconds, position.y);
        this.addDataPoint(this.kalmanPositionData.z, timeInSeconds, position.z);
        
        // Update charts
        this.updateChart('x', this.kalmanPositionData.x, 1);
        this.updateChart('y', this.kalmanPositionData.y, 1);
        this.updateChart('z', this.kalmanPositionData.z, 1);
    }
    
    addDataPoint(dataArray, time, value) {
        dataArray.push({ x: time, y: value });
        
        // Limit data points to prevent memory issues
        if (dataArray.length > this.maxDataPoints) {
            dataArray.shift();
        }
    }
    
    updateChart(axis, dataArray, datasetIndex) {
        const chart = this.charts[axis];
        if (!chart) return;
        
        // Update dataset
        chart.data.datasets[datasetIndex].data = [...dataArray];
        
        // Auto-scale X axis to show last 20 seconds
        if (dataArray.length > 0) {
            const latestTime = dataArray[dataArray.length - 1].x;
            const minTime = Math.max(0, latestTime - 20);
            
            chart.options.scales.x.min = minTime;
            chart.options.scales.x.max = latestTime + 2;
        }
        
        chart.update('none'); // No animation for better performance
    }
    
    clearData() {
        // Clear all data arrays
        this.statePositionData.x.length = 0;
        this.statePositionData.y.length = 0;
        this.statePositionData.z.length = 0;
        this.kalmanPositionData.x.length = 0;
        this.kalmanPositionData.y.length = 0;
        this.kalmanPositionData.z.length = 0;
        
        // Reset start time
        this.startTime = Date.now();
        
        // Update all charts
        Object.values(this.charts).forEach(chart => {
            chart.data.datasets.forEach(dataset => {
                dataset.data.length = 0;
            });
            chart.update();
        });
    }
}

// Global instance
let positionPlots;

// Initialize when DOM is loaded
document.addEventListener('DOMContentLoaded', () => {
    positionPlots = new PositionPlots();
});

// Global functions for HTML interface
function clearPlots() {
    if (positionPlots) {
        positionPlots.clearData();
    }
}

// Export for potential module use
if (typeof module !== 'undefined' && module.exports) {
    module.exports = { PositionPlots };
}
