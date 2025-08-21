// GPS Map functionality using Leaflet and OpenStreetMap
class GPSMap {
    constructor() {
        this.map = null;
        this.homeMarker = null;
        this.currentMarker = null;
        this.homePosition = { latitude: 0, longitude: 0 };
        this.currentPosition = { latitude: 0, longitude: 0 };
        this.initialized = false;
    }

    init() {
        if (this.initialized) return;

        const mapContainer = document.getElementById('gps-map');
        if (!mapContainer) {
            console.error('GPS map container not found');
            return;
        }

        try {
            // Initialize the map with a default view
            this.map = L.map('gps-map', {
                zoomControl: false,
                attributionControl: false
            }).setView([0, 0], 2);

            // Add OpenStreetMap tiles
            L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
                maxZoom: 19,
                attribution: '© OpenStreetMap contributors'
            }).addTo(this.map);

            // Create custom icons
            const homeIcon = L.divIcon({
                className: 'gps-marker home-marker',
                html: '<div class="marker-icon">🏠</div>',
                iconSize: [20, 20],
                iconAnchor: [10, 10]
            });

            const currentIcon = L.divIcon({
                className: 'gps-marker current-marker',
                html: '<div class="marker-icon">🚀</div>',
                iconSize: [20, 20],
                iconAnchor: [10, 10]
            });

            // Create markers (initially at 0,0)
            this.homeMarker = L.marker([0, 0], { icon: homeIcon }).addTo(this.map);
            this.currentMarker = L.marker([0, 0], { icon: currentIcon }).addTo(this.map);

            this.homeMarker.bindPopup('Home Position');
            this.currentMarker.bindPopup('Current Position');

            this.initialized = true;
            console.log('GPS map initialized');
        } catch (error) {
            console.error('Error initializing GPS map:', error);
        }
    }

    updateHomePosition(latitude, longitude) {
        if (!this.initialized || !this.map) return;

        this.homePosition = { latitude, longitude };
        
        if (this.homeMarker) {
            this.homeMarker.setLatLng([latitude, longitude]);
            this.homeMarker.setPopupContent(`Home: ${latitude.toFixed(6)}, ${longitude.toFixed(6)}`);
        }

        this.updateMapView();
    }

    updateCurrentPosition(latitude, longitude) {
        if (!this.initialized || !this.map) return;

        this.currentPosition = { latitude, longitude };
        
        if (this.currentMarker) {
            this.currentMarker.setLatLng([latitude, longitude]);
            this.currentMarker.setPopupContent(`Current: ${latitude.toFixed(6)}, ${longitude.toFixed(6)}`);
        }

        this.updateMapView();
    }

    updateMapView() {
        if (!this.initialized || !this.map) return;

        // If both positions are valid, fit the map to show both markers
        if (this.homePosition.latitude !== 0 && this.homePosition.longitude !== 0 &&
            this.currentPosition.latitude !== 0 && this.currentPosition.longitude !== 0) {
            
            const group = new L.featureGroup([this.homeMarker, this.currentMarker]);
            this.map.fitBounds(group.getBounds().pad(0.1));
        } 
        // If only current position is valid, center on it
        else if (this.currentPosition.latitude !== 0 && this.currentPosition.longitude !== 0) {
            this.map.setView([this.currentPosition.latitude, this.currentPosition.longitude], 15);
        }
        // If only home position is valid, center on it
        else if (this.homePosition.latitude !== 0 && this.homePosition.longitude !== 0) {
            this.map.setView([this.homePosition.latitude, this.homePosition.longitude], 15);
        }
    }

    resize() {
        if (this.initialized && this.map) {
            setTimeout(() => {
                this.map.invalidateSize();
            }, 100);
        }
    }
}

// Global instance
const gpsMap = new GPSMap();

// Initialize GPS map when DOM is loaded
document.addEventListener('DOMContentLoaded', () => {
    // Delay initialization to ensure Leaflet is loaded
    setTimeout(() => {
        gpsMap.init();
    }, 100);
});

// Export for use in other modules
if (typeof module !== 'undefined' && module.exports) {
    module.exports = { GPSMap, gpsMap };
}
