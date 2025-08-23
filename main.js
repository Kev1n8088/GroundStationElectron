const { app, BrowserWindow, dialog, ipcMain } = require('electron');
const path = require('path');
const fs = require('fs');
const { NtripClient } = require('ntrip-client');


// NTRIP client functionality
//let NtripClient = null;
let ntripClient = null;
let ntripConfig = null;

// Try to load NTRIP client - install with: npm install ntrip-client
try {
    NtripClient = require('ntrip-client');
} catch (error) {
    console.log('NTRIP client not available. Install with: npm install ntrip-client');
}

// Load NTRIP configuration
function loadNtripConfig() {
    try {
        const configPath = path.join(__dirname, 'ntrip-config.ini');
        const configData = fs.readFileSync(configPath, 'utf8');
        
        // Parse INI file
        const config = {
            host: '',
            port: 9000,
            mountpoint: '',
            username: '',
            password: '',
            xyz: [0, 0, 0],
            interval: 2000
        };
        
        const lines = configData.split('\n');
        let currentSection = '';
        
        for (const line of lines) {
            const trimmed = line.trim();
            if (trimmed.startsWith('[') && trimmed.endsWith(']')) {
                currentSection = trimmed.slice(1, -1);
            } else if (trimmed.includes('=')) {
                const [key, value] = trimmed.split('=').map(s => s.trim());
                
                if (currentSection === 'NTRIP') {
                    if (key === 'host') config.host = value;
                    else if (key === 'port') config.port = parseInt(value);
                    else if (key === 'mountpoint') config.mountpoint = value;
                    else if (key === 'username') config.username = value;
                    else if (key === 'password') config.password = value;
                } else if (currentSection === 'POSITION') {
                    if (key === 'x') config.xyz[0] = parseFloat(value);
                    else if (key === 'y') config.xyz[1] = parseFloat(value);
                    else if (key === 'z') config.xyz[2] = parseFloat(value);
                } else if (currentSection === 'OPTIONS') {
                    if (key === 'interval') config.interval = parseInt(value);
                }
            }
        }
        
        ntripConfig = config;
        console.log('NTRIP config loaded:', config);
        return config;
    } catch (error) {
        console.error('Failed to load NTRIP config:', error);
        return null;
    }
}


app.commandLine.appendSwitch('ignore-gpu-blocklist');
app.commandLine.appendSwitch('disable-gpu-shader-disk-cache');
app.commandLine.appendSwitch('enable-gpu-rasterization');
app.commandLine.appendSwitch('enable-zero-copy');
app.commandLine.appendSwitch('use-angle', 'd3d11'); // or 'gl' to try OpenGL
app.commandLine.appendSwitch('high-dpi-support', 1)
app.commandLine.appendSwitch('force-device-scale-factor', 1)
// On Optimus laptops:
app.commandLine.appendSwitch('force_high_performance_gpu');


app.on("ready", () => {
    console.log("hello")
    console.log(app.getGPUFeatureStatus());
  });

let mainWindow;

//app.disableHardwareAcceleration(false);

function createWindow() {
    mainWindow = new BrowserWindow({
        width: 2304,
        height: 1440,
        minWidth: 2304,
        minHeight: 1440,
        webPreferences: {
            nodeIntegration: true,
            contextIsolation: false,
            enableRemoteModule: true
        },
        icon: path.join(__dirname, 'assets', 'icon.png'), // Optional icon
        show: false // Don't show until ready
    });

    mainWindow.loadFile('index.html');

    // Show window when ready
    mainWindow.once('ready-to-show', () => {
        mainWindow.show();
        // Trigger OBJ model load on startup
    });

    // Handle window closed
    mainWindow.on('closed', () => {
        mainWindow = null;
    });

    // Open DevTools in development
    if (process.argv.includes('--dev')) {
        mainWindow.webContents.openDevTools();
    }

    // Maintain 16:10 aspect ratio
    mainWindow.on('resize', () => {
        const [width, height] = mainWindow.getSize();
        const aspectRatio = 16 / 10;
        
        if (Math.abs(width / height - aspectRatio) > 0.01) {
            const newHeight = Math.round(width / aspectRatio);
            mainWindow.setSize(width, newHeight);
        }
    });
}


// NTRIP Client Management
function startNtripClient() {
    //console.log('attempted ntrip start');
    if (!NtripClient || !ntripConfig) {
        console.log('NTRIP client or config not available');
        return false;
    }
    
    if (ntripClient) {
        console.log('NTRIP client already running');
        return true;
    }
    
    try {
        ntripClient = new NtripClient(ntripConfig);
        
        ntripClient.on('data', (data) => {
            console.log(`RTCM Data received: ${data.length} bytes`);
            
            // Send RTCM data to renderer process
            if (mainWindow && mainWindow.webContents) {
                mainWindow.webContents.send('rtcm-data', {
                    timestamp: new Date().toISOString(),
                    length: data.length,
                    data: Array.from(data) // Convert Buffer to array for JSON serialization
                });
            }
        });
        
        ntripClient.on('connect', () => {
            console.log('NTRIP client connected successfully');
            if (mainWindow && mainWindow.webContents) {
                mainWindow.webContents.send('ntrip-status', { connected: true });
            }
        });
        
        ntripClient.on('close', () => {
            console.log('NTRIP client connection closed');
            ntripClient = null;
            if (mainWindow && mainWindow.webContents) {
                mainWindow.webContents.send('ntrip-status', { connected: false });
            }
        });
        
        ntripClient.on('error', (err) => {
            console.error('NTRIP client error:', err);
            ntripClient = null;
            if (mainWindow && mainWindow.webContents) {
                mainWindow.webContents.send('ntrip-status', { connected: false, error: err.message });
            }
        });
        
        // Start connection
        ntripClient.run();
        
        console.log('NTRIP client started');
        return true;
    } catch (error) {
        console.error('Failed to start NTRIP client:', error);
        return false;
    }
}

function stopNtripClient() {
    if (ntripClient) {
        try {
            ntripClient.close();
        } catch (error) {
            console.error('Error disconnecting NTRIP client:', error);
        }
        ntripClient = null;
        console.log('NTRIP client stopped');
        if (mainWindow && mainWindow.webContents) {
            mainWindow.webContents.send('ntrip-status', { connected: false });
        }
        return true;
    }
    return false;
}

// IPC Handlers
ipcMain.handle('ntrip-start', async () => {
    loadNtripConfig();
    return startNtripClient();
});

ipcMain.handle('ntrip-stop', async () => {
    return stopNtripClient();
});

ipcMain.handle('ntrip-status', async () => {
    return {
        available: !!NtripClient,
        connected: !!ntripClient,
        config: ntripConfig
    };
});

// Handle reading OBJ file
ipcMain.handle('read-obj-file', async () => {
    try {
        const filePath = path.join(__dirname, 'model.obj'); // Ensure correct path
        const data = fs.readFileSync(filePath, 'utf8');
        return { success: true, data, filename: path.basename(filePath) };
    } catch (error) {
        return { success: false, error: error.message };
    }
});

// App event handlers
app.whenReady().then(() => {
    loadNtripConfig();
    createWindow();
});

app.on('window-all-closed', () => {
    if (process.platform !== 'darwin') {
        app.quit();
    }
});

app.on('activate', () => {
    if (BrowserWindow.getAllWindows().length === 0) {
        createWindow();
    }
});

// Handle certificate errors for development
app.on('certificate-error', (event, webContents, url, error, certificate, callback) => {
    if (process.argv.includes('--dev')) {
        event.preventDefault();
        callback(true);
    } else {
        callback(false);
    }
});