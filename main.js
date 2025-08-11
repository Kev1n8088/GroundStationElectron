const { app, BrowserWindow, dialog, ipcMain } = require('electron');
const path = require('path');
const fs = require('fs');


app.commandLine.appendSwitch('ignore-gpu-blocklist');
app.commandLine.appendSwitch('disable-gpu-shader-disk-cache');
app.commandLine.appendSwitch('enable-gpu-rasterization');
app.commandLine.appendSwitch('enable-zero-copy');
app.commandLine.appendSwitch('use-angle', 'd3d11'); // or 'gl' to try OpenGL

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
        width: 1400,
        height: 900,
        minWidth: 1200,
        minHeight: 700,
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
    });

    // Handle window closed
    mainWindow.on('closed', () => {
        mainWindow = null;
    });

    // Open DevTools in development
    if (process.argv.includes('--dev')) {
        mainWindow.webContents.openDevTools();
    }
}


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
app.whenReady().then(createWindow);

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