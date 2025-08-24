// Three.js scene management
let scene, camera, renderer, vehicle, ground;
let vehicleState = {
    position: { x: 0, y: 0, z: 0 }, // in vehicle convention, x vertical
    quaternion: { x: 0, y: 0, z: 0, w: 1 },
};

class ThreeScene {
    constructor() {
        this.initialized = false;
        this.vehicle = null;
        this.arrowHelpers = [];
    }

    init() {
        const container = document.getElementById('threejs-container');
        if (!container) {
            console.error('Three.js container not found');
            return false;
        }

        // Scene setup
        scene = new THREE.Scene();
        scene.background = new THREE.Color(0x001122);

        // Camera setup
        camera = new THREE.PerspectiveCamera(
            75, 
            container.clientWidth / container.clientHeight, 
            0.1, 
            1000
        );
        camera.position.set(-3, 3, -3);
        camera.lookAt(0, 0, 0);

        // Renderer setup
        renderer = new THREE.WebGLRenderer({ 
            antialias: true,
            alpha: true 
        });
        renderer.setSize(container.clientWidth, container.clientHeight);
        renderer.shadowMap.enabled = true;
        renderer.shadowMap.type = THREE.PCFSoftShadowMap;
        renderer.setClearColor(0x001122, 1);
        
        container.appendChild(renderer.domElement);

        this.setupLighting();
        this.createGroundPlane();
        this.createOriginMarker();
        this.createDefaultVehicle();
        this.setupControls();

        // Window resize handling
        window.addEventListener('resize', () => this.onWindowResize());

        this.initialized = true;
        return true;
    }

    setupLighting() {
        // Ambient light
        const ambientLight = new THREE.AmbientLight(0x404040, 0.6);
        scene.add(ambientLight);

        // Directional light (sun)
        const directionalLight = new THREE.DirectionalLight(0xffffff, 0.8);
        directionalLight.position.set(20, 20, 10);
        directionalLight.castShadow = true;
        directionalLight.shadow.mapSize.width = 2048;
        directionalLight.shadow.mapSize.height = 2048;
        directionalLight.shadow.camera.near = 0.5;
        directionalLight.shadow.camera.far = 50;
        directionalLight.shadow.camera.left = -25;
        directionalLight.shadow.camera.right = 25;
        directionalLight.shadow.camera.top = 25;
        directionalLight.shadow.camera.bottom = -25;
        scene.add(directionalLight);

        // Point light for additional illumination
        const pointLight = new THREE.PointLight(0x4080ff, 0.4, 30);
        pointLight.position.set(-10, 10, -10);
        scene.add(pointLight);
    }

    createGroundPlane() {
        // Ground plane
        const groundGeometry = new THREE.PlaneGeometry(100, 100);
        const groundMaterial = new THREE.MeshLambertMaterial({ 
            color: 0x2d5d2d,
            transparent: true,
            opacity: 0.8
        });
        ground = new THREE.Mesh(groundGeometry, groundMaterial);
        ground.rotation.x = -Math.PI / 2;
        ground.receiveShadow = true;
        ground.position.set(0, -0.1, 0);
        scene.add(ground);

        // Grid helper
        const gridHelper = new THREE.GridHelper(100, 100, 0x444444, 0x444444);
        gridHelper.position.set(0, -0.1, 0);
        scene.add(gridHelper);

        // Axes helper at origin
        const axesHelper = new THREE.AxesHelper(2);
        axesHelper.rotation.z = Math.PI / 2;

        //change color to match falcon9 payload guideline reference color
        const colors = [
            new THREE.Color(0x00ff00), // X axis
            new THREE.Color(0xff0000), // Y axis 
            new THREE.Color(0x0000ff), // Z axis 
        ];
        
        for (let i = 0; i < 3; i++) {
            axesHelper.geometry.attributes.color.setXYZ(i * 2, colors[i].r, colors[i].g, colors[i].b);
            axesHelper.geometry.attributes.color.setXYZ(i * 2 + 1, colors[i].r, colors[i].g, colors[i].b);
        }
        axesHelper.geometry.attributes.color.needsUpdate = true;

        scene.add(axesHelper);

        // Add a label for the axes

        const xLabel = this.createTextSprite('+X (U)', 1.0, '#00ff00');
        xLabel.position.set(0, 2.1, 0);
        scene.add(xLabel);

        const yLabel = this.createTextSprite('+Y (E)', 1.0, '#ff0000');
        yLabel.position.set(-2.1, 0, 0);
        scene.add(yLabel);
        
        
        const zLabel = this.createTextSprite('+Z (N)', 1.0, '#0000ff');
        zLabel.position.set(0, 0, 2.1);
        scene.add(zLabel);
    }

    createOriginMarker() {
        // Origin marker
        const originGeometry = new THREE.SphereGeometry(0.1, 16, 16);
        const originMaterial = new THREE.MeshLambertMaterial({ 
            color: 0xff0000,
            emissive: 0x220000
        });
        const origin = new THREE.Mesh(originGeometry, originMaterial);
        origin.position.set(0, 0, 0);
        origin.castShadow = true;
        scene.add(origin);

        // // Origin label (optional)
        // const originLabel = this.createTextSprite('ORIGIN', 2.0);
        // originLabel.position.set(0, 1, 0);
        // scene.add(originLabel);
    }

    createDefaultVehicle() {
        const vehicleGroup = new THREE.Group();

        // Main body (fuselage)
        const bodyGeometry = new THREE.CylinderGeometry(0.1, 0.1, 0.4, 12);
        const bodyMaterial = new THREE.MeshLambertMaterial({ 
            color: 0x00ff00,
            emissive: 0x002200
        });
        const body = new THREE.Mesh(bodyGeometry, bodyMaterial);
        body.rotation.z = Math.PI / 2;
        body.castShadow = true;
        vehicleGroup.add(body);

        vehicle = vehicleGroup;
        vehicle.position.copy(vehicleState.position);
        vehicle.castShadow = true;
        scene.add(vehicle);

        this.vehicle = vehicle;
    }

    createTextSprite(text, scale = 1, color = '#ffffff') {
        const canvas = document.createElement('canvas');
        const context = canvas.getContext('2d');
        context.font = '64px Futura Md BT';
        context.fillStyle = color;
        context.fillText(text, 0, 64);

        const texture = new THREE.CanvasTexture(canvas);
        const spriteMaterial = new THREE.SpriteMaterial({ map: texture });
        const sprite = new THREE.Sprite(spriteMaterial);
        sprite.scale.set(scale, scale * 0.5, 1);
        
        return sprite;
    }

    setupControls() {
        // Basic mouse controls for camera
        let isMouseDown = false;
        let mouseX = 0, mouseY = 0;
        let cameraRadius = 4.2;
        let cameraTheta = Math.PI + Math.PI / 4;
        let cameraPhi = Math.PI / 4;

        const container = document.getElementById('threejs-container');
        
        container.addEventListener('mousedown', (event) => {
            isMouseDown = true;
            mouseX = event.clientX;
            mouseY = event.clientY;
        });

        container.addEventListener('mouseup', () => {
            isMouseDown = false;
        });

        container.addEventListener('mousemove', (event) => {
            if (!isMouseDown) return;

            const deltaX = event.clientX - mouseX;
            const deltaY = event.clientY - mouseY;

            cameraTheta += deltaX * 0.01;
            cameraPhi -= deltaY * 0.01;
            cameraPhi = Math.max(0.1, Math.min(Math.PI - 0.1, cameraPhi));

            this.updateCameraPosition(cameraRadius, cameraTheta, cameraPhi);

            mouseX = event.clientX;
            mouseY = event.clientY;
        });

        container.addEventListener('wheel', (event) => {
            cameraRadius += event.deltaY * 0.01;
            cameraRadius = Math.max(1, Math.min(50, cameraRadius));
            this.updateCameraPosition(cameraRadius, cameraTheta, cameraPhi);
        });
    }

    updateCameraPosition(radius, theta, phi) {
        camera.position.x = radius * Math.sin(phi) * Math.cos(theta);
        camera.position.y = radius * Math.cos(phi);
        camera.position.z = radius * Math.sin(phi) * Math.sin(theta);
        camera.lookAt(0, 0, 0);
    }

    updateVehicleTransform() {
        if (!this.vehicle) return;

        // Update position
        this.vehicle.position.set(-vehicleState.position.y, vehicleState.position.x, vehicleState.position.z);

        const upOffset = new THREE.Quaternion();
        upOffset.setFromAxisAngle(new THREE.Vector3(0, 0, 1), Math.PI / 2); // rotate -90° about X to map +Z to +Y

        const q = new THREE.Quaternion(vehicleState.quaternion.x, vehicleState.quaternion.y, vehicleState.quaternion.z, vehicleState.quaternion.w);
    

        const finalQuat = q.clone().premultiply(upOffset);
        this.vehicle.quaternion.copy(finalQuat);

    }

    replaceVehicle(newVehicle) {
        if (this.vehicle) {
            scene.remove(this.vehicle);
        }
        
        this.vehicle = newVehicle;
        this.vehicle.position.copy(vehicleState.position);
        scene.add(this.vehicle);
        
        // Update global reference
        vehicle = this.vehicle;
    }

    onWindowResize() {
        const container = document.getElementById('threejs-container');
        if (!container) return;

        camera.aspect = container.clientWidth / container.clientHeight;
        camera.updateProjectionMatrix();
        renderer.setSize(container.clientWidth, container.clientHeight);
    }

    render() {
        if (!this.initialized) return;
        renderer.render(scene, camera);
    }
}

// Global instance
const threeScene = new ThreeScene();