// Model loading functionality
const { ipcRenderer } = require('electron');

// Listen for startup model loading
ipcRenderer.on('load-startup-model', () => {
    loadObjModel();
});

class ModelLoader {
    constructor() {
        this.currentModel = null;
        this.defaultScale = 1.0;
    }

    async loadObjFile() {
        try {
            // Show loading indicator
            this.showLoadingIndicator();

            // Read the file content
            const result = await ipcRenderer.invoke('read-obj-file');
            console.log(result.data);
            
            if (!result.success) {
                alert('Failed to read OBJ file: ' + result.error);
                this.hideLoadingIndicator();
                return;
            }

            // Load the OBJ data using Three.js OBJLoader
            await this.parseAndLoadObj(result.data, result.filename);
            
        } catch (error) {
            console.error('Error loading OBJ file:', error);
            alert('Error loading model: ' + error.message);
            this.hideLoadingIndicator();
        }
    }

    async parseAndLoadObj(objData, filename) {
        return new Promise((resolve, reject) => {
            const loader = new THREE.OBJLoader(); // Ensure this uses the globally available OBJLoader
            
            // Parse the OBJ data
            const object = loader.parse(objData);
            try {
                console.log('OBJ loaded successfully:', object);
                
                // Process the loaded model
                this.processLoadedModel(object, filename);
                this.hideLoadingIndicator();
                resolve(object);
                
            } catch (error) {
                console.error('Error processing loaded model:', error);
                reject(error);
            }
        });
    }

    processLoadedModel(object, filename) {
        // Create a group to contain the model
        const modelGroup = new THREE.Group();
        
        // Add all children from the loaded object
        object.children.forEach(child => {
            modelGroup.add(child.clone());
        });

        // Calculate bounding box for auto-scaling
        const box = new THREE.Box3().setFromObject(modelGroup);
        const size = box.getSize(new THREE.Vector3());
        const maxDimension = Math.max(size.x, size.y, size.z);
        
        // Auto-scale to reasonable size (target size around 3 units)
        const targetSize = 0.5;
        const autoScale = targetSize / maxDimension;
        
        console.log('Model dimensions:', size);
        console.log('Auto-scale factor:', autoScale);
        
        modelGroup.scale.setScalar(autoScale);
        this.defaultScale = autoScale;

        // Center the model
        const center = box.getCenter(new THREE.Vector3());
        modelGroup.position.sub(center.multiplyScalar(autoScale));
        //modelGroup.rotation.y = -Math.PI / 2;

        modelGroup.traverse(child => {
            if (child.isMesh) {
                // Rotate geometry -90 degrees around X to convert Z-up to Y-up
                child.geometry.applyMatrix4(new THREE.Matrix4().makeRotationZ(-Math.PI / 2));
                child.geometry.computeBoundingBox();
                child.geometry.computeVertexNormals();
            }
        });
        
        
        modelGroup.traverse(child => {
            if (child.isMesh) {
                // Rotate geometry -90 degrees around X to convert Z-up to Y-up
                child.geometry.applyMatrix4(new THREE.Matrix4().makeRotationX(-Math.PI / 2));
                child.geometry.computeBoundingBox();
                child.geometry.computeVertexNormals();
            }
        });

        // Apply materials if needed
        this.applyDefaultMaterials(modelGroup);

        // Enable shadows
        modelGroup.traverse(child => {
            if (child.isMesh) {
                child.castShadow = true;
                child.receiveShadow = true;
            }
        });

        // Replace the current vehicle
        threeScene.replaceVehicle(modelGroup);
        console.log('Model loaded and processed successfully');
    }

    applyDefaultMaterials(object) {
        const defaultMaterial = new THREE.MeshLambertMaterial({ 
            color: 0x00ff00,
            emissive: 0x002200
        });

        object.traverse(child => {
            if (child.isMesh) {
                if (!child.material || child.material.length === 0) {
                    child.material = defaultMaterial;
                } else {
                    // Enhance existing materials
                    if (Array.isArray(child.material)) {
                        child.material.forEach(mat => {
                            if (mat.isMeshBasicMaterial) {
                                // Convert basic materials to Lambert for better lighting
                                const newMat = new THREE.MeshLambertMaterial();
                                newMat.copy(mat);
                                mat = newMat;
                            }
                        });
                    } else {
                        if (child.material.isMeshBasicMaterial) {
                            const newMat = new THREE.MeshLambertMaterial();
                            newMat.copy(child.material);
                            child.material = newMat;
                        }
                    }
                }
            }
        });
    }

    showLoadingIndicator() {
        const container = document.getElementById('threejs-container');
        const existing = container.querySelector('.loading');
        if (existing) return;

        const loadingDiv = document.createElement('div');
        loadingDiv.className = 'loading';
        loadingDiv.innerHTML = 'Loading 3D Model...';
        container.appendChild(loadingDiv);
    }

    hideLoadingIndicator() {
        const container = document.getElementById('threejs-container');
        const loading = container.querySelector('.loading');
        if (loading) {
            loading.remove();
        }
    }

    updateModelStatus(filename, success) {
        const statusElement = document.getElementById('model-status');
        if (success) {
            statusElement.textContent = `Model: ${filename}`;
            statusElement.className = 'model-status loaded';
        } else {
            statusElement.textContent = 'Model: FAILED';
            statusElement.className = 'model-status';
        }
    }

    // Helper method to create a simple test OBJ string
    createTestObjData() {
        return `
# Simple test aircraft OBJ
v -1.0 0.0 0.0
v 1.0 0.0 0.0
v 0.0 0.0 -2.0
v 0.0 0.5 0.0

f 1 2 4
f 2 3 4
f 3 1 4
f 1 3 2
`;
    }

    // Method to load the test model
    loadTestModel() {
        const testData = this.createTestObjData();
        this.parseAndLoadObj(testData, 'test_aircraft.obj');
    }
}

// Global functions for HTML interface
const modelLoader = new ModelLoader();

async function loadObjModel() {
    await modelLoader.loadObjFile();
}

function updateModelScale() {
    modelLoader.updateModelScale();
}

// Optional: Load test model function
function loadTestModel() {
    modelLoader.loadTestModel();
}
