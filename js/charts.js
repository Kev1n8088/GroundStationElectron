// Chart management
let chart1, chart2;
let chartData1 = [];
let chartData2 = [];
const MAX_CHART_POINTS = 30;

class ChartManager {
    constructor() {
        this.charts = {};
        this.initialized = false;
    }

    init() {
        this.createAltitudeChart();
        this.createOrientationChart();
        this.initialized = true;
    }

    createAltitudeChart() {
        const ctx1 = document.getElementById('chart1').getContext('2d');
        
        chart1 = new Chart(ctx1, {
            type: 'line',
            data: {
                labels: [],
                datasets: [{
                    label: 'Altitude (m)',
                    data: chartData1,
                    borderColor: '#00ff00',
                    backgroundColor: 'rgba(0, 255, 0, 0.1)',
                    tension: 0.4,
                    fill: true,
                    pointRadius: 2,
                    pointHoverRadius: 4,
                    borderWidth: 2
                }]
            },
            options: {
                responsive: true,
                maintainAspectRatio: false,
                animation: {
                    duration: 0 // Disable animations for real-time performance
                },
                interaction: {
                    intersect: false,
                    mode: 'index'
                },
                scales: {
                    y: { 
                        beginAtZero: false,
                        grid: { 
                            color: '#30363d',
                            borderColor: '#30363d'
                        },
                        ticks: { 
                            color: '#00ff00',
                            font: { size: 10 }
                        },
                        title: {
                            display: true,
                            text: 'Height (m)',
                            color: '#00ff00',
                            font: { size: 11 }
                        }
                    },
                    x: { 
                        grid: { 
                            color: '#30363d',
                            borderColor: '#30363d'
                        },
                        ticks: { 
                            color: '#00ff00',
                            font: { size: 9 },
                            maxTicksLimit: 6
                        },
                        title: {
                            display: true,
                            text: 'Time',
                            color: '#00ff00',
                            font: { size: 11 }
                        }
                    }
                },
                plugins: {
                    legend: { 
                        labels: { 
                            color: '#00ff00',
                            font: { size: 11 }
                        }
                    },
                    tooltip: {
                        backgroundColor: 'rgba(0, 0, 0, 0.8)',
                        titleColor: '#00ff00',
                        bodyColor: '#00ff00',
                        borderColor: '#30363d',
                        borderWidth: 1
                    }
                }
            }
        });

        this.charts.altitude = chart1;
    }

    createOrientationChart() {
        const ctx2 = document.getElementById('chart2').getContext('2d');
        
        chart2 = new Chart(ctx2, {
            type: 'line',
            data: {
                labels: [],
                datasets: [
                    {
                        label: 'Roll (°)',
                        data: [],
                        borderColor: '#ff6b6b',
                        backgroundColor: 'rgba(255, 107, 107, 0.1)',
                        tension: 0.4,
                        pointRadius: 1,
                        pointHoverRadius: 3,
                        borderWidth: 2
                    },
                    {
                        label: 'Pitch (°)',
                        data: [],
                        borderColor: '#4ecdc4',
                        backgroundColor: 'rgba(78, 205, 196, 0.1)',
                        tension: 0.4,
                        pointRadius: 1,
                        pointHoverRadius: 3,
                        borderWidth: 2
                    },
                    {
                        label: 'Yaw (°)',
                        data: [],
                        borderColor: '#ffe66d',
                        backgroundColor: 'rgba(255, 230, 109, 0.1)',
                        tension: 0.4,
                        pointRadius: 1,
                        pointHoverRadius: 3,
                        borderWidth: 2
                    }
                ]
            },
            options: {
                responsive: true,
                maintainAspectRatio: false,
                animation: {
                    duration: 0
                },
                interaction: {
                    intersect: false,
                    mode: 'index'
                },
                scales: {
                    y: { 
                        grid: { 
                            color: '#30363d',
                            borderColor: '#30363d'
                        },
                        ticks: { 
                            color: '#00ff00',
                            font: { size: 10 }
                        },
                        title: {
                            display: true,
                            text: 'Degrees (°)',
                            color: '#00ff00',
                            font: { size: 11 }
                        },
                        min: -180,
                        max: 180
                    },
                    x: { 
                        grid: { 
                            color: '#30363d',
                            borderColor: '#30363d'
                        },
                        ticks: { 
                            color: '#00ff00',
                            font: { size: 9 },
                            maxTicksLimit: 6
                        },
                        title: {
                            display: true,
                            text: 'Time',
                            color: '#00ff00',
                            font: { size: 11 }
                        }
                    }
                },
                plugins: {
                    legend: { 
                        labels: { 
                            color: '#00ff00',
                            font: { size: 11 }
                        }
                    },
                    tooltip: {
                        backgroundColor: 'rgba(0, 0, 0, 0.8)',
                        titleColor: '#00ff00',
                        bodyColor: '#ffffff',
                        borderColor: '#30363d',
                        borderWidth: 1
                    }
                }
            }
        });

        this.charts.orientation = chart2;
    }

    updateCharts() {
        if (!this.initialized) return;

        const now = Date.now();
        const timeLabel = new Date(now).toLocaleTimeString('en-US', {
            hour12: false,
            minute: '2-digit',
            second: '2-digit'
        });

        // Update altitude chart
        this.updateAltitudeChart(timeLabel);
        
        // Update orientation chart
        this.updateOrientationChart(timeLabel);
    }

    updateAltitudeChart(timeLabel) {
        chartData1.push(vehicleState.position.y);
        chart1.data.labels.push(timeLabel);

        // Keep only the last N points
        if (chartData1.length > MAX_CHART_POINTS) {
            chartData1.shift();
            chart1.data.labels.shift();
        }

        chart1.data.datasets[0].data = chartData1;
        chart1.update('none'); // Use 'none' mode for better performance
    }

    updateOrientationChart(timeLabel) {
        chart2.data.labels.push(timeLabel);
        chart2.data.datasets[0].data.push(vehicleState.euler.roll);
        chart2.data.datasets[1].data.push(vehicleState.euler.pitch);
        chart2.data.datasets[2].data.push(vehicleState.euler.yaw);

        // Keep only the last N points
        if (chart2.data.labels.length > MAX_CHART_POINTS) {
            chart2.data.labels.shift();
            chart2.data.datasets.forEach(dataset => dataset.data.shift());
        }

        chart2.update('none');
    }

    // Method to add custom data points
    addDataPoint(chartName, value, label = null) {
        const chart = this.charts[chartName];
        if (!chart) return;

        if (!label) {
            label = new Date().toLocaleTimeString('en-US', {
                hour12: false,
                minute: '2-digit',
                second: '2-digit'
            });
        }

        chart.data.labels.push(label);
        chart.data.datasets[0].data.push(value);

        // Keep only the last N points
        if (chart.data.labels.length > MAX_CHART_POINTS) {
            chart.data.labels.shift();
            chart.data.datasets[0].data.shift();
        }

        chart.update('none');
    }

    // Method to clear all chart data
    clearAllCharts() {
        Object.values(this.charts).forEach(chart => {
            chart.data.labels = [];
            chart.data.datasets.forEach(dataset => {
                dataset.data = [];
            });
            chart.update();
        });

        // Clear data arrays
        chartData1.length = 0;
        chartData2.length = 0;
    }

    // Method to export chart data
    exportChartData() {
        const data = {
            timestamp: new Date().toISOString(),
            altitude: {
                labels: [...chart1.data.labels],
                data: [...chartData1]
            },
            orientation: {
                labels: [...chart2.data.labels],
                roll: [...chart2.data.datasets[0].data],
                pitch: [...chart2.data.datasets[1].data],
                yaw: [...chart2.data.datasets[2].data]
            }
        };

        return JSON.stringify(data, null, 2);
    }

    // Method to resize charts when window changes
    resize() {
        Object.values(this.charts).forEach(chart => {
            chart.resize();
        });
    }
}

// Global instance
const chartManager = new ChartManager();

// Global function for updating charts (called from other modules)
function updateCharts() {
    chartManager.updateCharts();
}

// Global function for clearing charts
function clearCharts() {
    chartManager.clearAllCharts();
}

// Handle window resize
window.addEventListener('resize', () => {
    if (chartManager.initialized) {
        chartManager.resize();
    }
});