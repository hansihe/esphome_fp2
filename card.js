/**
 * Aqara FP2 Presence Sensor Card for Home Assistant
 *
 * Configuration options:
 * - entity_prefix: (required) The entity prefix for the FP2 sensor (e.g., "sensor.fp2_living_room")
 * - title: (optional) Card title
 * - display_mode: (optional) "full" or "zoomed" - Default: "full"
 * - show_grid: (optional) Show grid lines - Default: true
 * - show_sensor_position: (optional) Show sensor position marker - Default: true
 * - show_zone_labels: (optional) Show zone labels - Default: true
 * - mounting_position: (optional) Sensor mounting position - Default: from entity or "wall"
 */
class AqaraFP2Card extends HTMLElement {
  constructor() {
    super();
    this.config = {};
    this.displayMode = "full"; // 'full' or 'zoomed'
    this.showGrid = true;
    this.showSensorPosition = true;
    this.showZoneLabels = true;
    this.gridSize = 14;
  }

  set hass(hass) {
    this._hass = hass;

    if (!this.content) {
      this.initializeCard();
    }

    this.updateCard();
  }

  setConfig(config) {
    if (!config.entity_prefix) {
      throw new Error("You need to define entity_prefix");
    }
    this.config = config;
    this.displayMode = config.display_mode || "full";
    this.showGrid = config.show_grid !== false;
    this.showSensorPosition = config.show_sensor_position !== false;
    this.showZoneLabels = config.show_zone_labels !== false;
    console.log(`[FP2 Card] Card configured with entity_prefix: ${config.entity_prefix}`);
  }

  initializeCard() {
    this.innerHTML = `
      <ha-card>
        <div class="card-header">
          <div class="name">${this.config.title || "Aqara FP2 Presence Sensor"}</div>
          <div class="controls">
            <button class="mode-toggle" title="Toggle Display Mode">
              <ha-icon icon="mdi:fit-to-screen"></ha-icon>
            </button>
          </div>
        </div>
        <div class="card-content">
          <canvas id="fp2-canvas"></canvas>
          <div class="info-panel"></div>
        </div>
      </ha-card>
      <style>
        ha-card {
          padding: 16px;
        }
        .card-header {
          display: flex;
          justify-content: space-between;
          align-items: center;
          margin-bottom: 16px;
        }
        .card-header .name {
          font-size: 24px;
          font-weight: 500;
        }
        .card-header .controls {
          display: flex;
          gap: 8px;
        }
        .card-header button {
          background: none;
          border: 1px solid var(--divider-color);
          border-radius: 4px;
          padding: 8px;
          cursor: pointer;
          color: var(--primary-text-color);
        }
        .card-header button:hover {
          background: var(--secondary-background-color);
        }
        .card-header button.active {
          background: var(--primary-color);
          color: var(--text-primary-color);
        }
        .card-content {
          display: flex;
          flex-direction: column;
          gap: 16px;
        }
        #fp2-canvas {
          width: 100%;
          height: auto;
          border: 1px solid var(--divider-color);
          border-radius: 4px;
          background: var(--card-background-color);
        }
        .info-panel {
          font-size: 14px;
          color: var(--secondary-text-color);
        }
      </style>
    `;

    this.content = this.querySelector(".card-content");
    this.canvas = this.querySelector("#fp2-canvas");
    this.ctx = this.canvas.getContext("2d");
    this.infoPanel = this.querySelector(".info-panel");

    // Set up event listeners
    this.querySelector(".mode-toggle").addEventListener("click", () => {
      this.displayMode = this.displayMode === "full" ? "zoomed" : "full";
      this.updateCard();
    });

    this.canvas.addEventListener("click", (e) => this.handleCanvasClick(e));
    this.canvas.addEventListener("mousemove", (e) => this.handleCanvasHover(e));
  }

  updateCard() {
    if (!this._hass || !this.canvas) {
      console.log(`[FP2 Card] updateCard skipped: hass=${!!this._hass}, canvas=${!!this.canvas}`);
      return;
    }

    console.log(`[FP2 Card] ===== Card update triggered =====`);
    const data = this.gatherEntityData();
    this.renderCanvas(data);
    this.updateInfoPanel(data);
  }

  gatherEntityData() {
    const prefix = this.config.entity_prefix;
    const hass = this._hass;

    console.log(`[FP2 Card] Starting entity data gathering with prefix: ${prefix}`);

    // Helper to safely get entity state
    const getEntityState = (entityId) => {
      const state = hass.states[entityId];
      if (state) {
        console.log(`[FP2 Card] ✓ Entity found: ${entityId} = "${state.state}"`);
        return state.state;
      } else {
        console.warn(`[FP2 Card] ✗ Entity not found: ${entityId}`);
        return null;
      }
    };

    // Helper to parse JSON attributes
    const getJsonAttribute = (entityId) => {
      const state = hass.states[entityId];
      if (!state) {
        console.warn(`[FP2 Card] ✗ JSON entity not found: ${entityId}`);
        return null;
      }
      try {
        const parsed = JSON.parse(state.state);
        console.log(`[FP2 Card] ✓ JSON entity parsed: ${entityId}`, parsed);
        return parsed;
      } catch (e) {
        console.error(`[FP2 Card] ✗ JSON parse error for ${entityId}:`, e.message, `Raw value: "${state.state}"`);
        return null;
      }
    };

    // Helper to get main device ID from an entity
    const getMainDeviceId = () => {
      // Try to find the main device by looking up one of the known entities
      const testEntity = `${prefix}_edge_label_grid`;
      if (hass.entities && hass.entities[testEntity]) {
        const deviceId = hass.entities[testEntity].device_id;
        console.log(`[FP2 Card] ✓ Found main device ID: ${deviceId}`);
        return deviceId;
      }
      console.warn(`[FP2 Card] ✗ Could not find main device (entity registry not available or entity not found)`);
      return null;
    };

    // Helper to find zone sub-devices
    const findZoneSubDevices = (mainDeviceId) => {
      if (!hass.devices || !hass.entities) {
        console.warn(`[FP2 Card] ✗ Device/Entity registry not available`);
        return [];
      }

      const subDevices = [];

      // Find all devices that have via_device_id pointing to main device
      Object.entries(hass.devices).forEach(([deviceId, device]) => {
        if (device.via_device_id === mainDeviceId) {
          console.log(`[FP2 Card] ✓ Found sub-device: ${deviceId} (${device.name || 'unnamed'})`);
          subDevices.push({ deviceId, device });
        }
      });

      console.log(`[FP2 Card] Total sub-devices found: ${subDevices.length}`);
      return subDevices;
    };

    // Helper to extract entity prefix from a device's entities
    const getDeviceEntityPrefix = (deviceId) => {
      // Find all entities for this device
      const deviceEntities = Object.entries(hass.entities)
        .filter(([entityId, entity]) => entity.device_id === deviceId)
        .map(([entityId, entity]) => entityId);

      // Look for a _map entity to determine the prefix
      const mapEntity = deviceEntities.find(id => id.endsWith('_map'));
      if (mapEntity) {
        // Extract prefix by removing the _map suffix and the zone identifier
        // Format: {prefix}_zone_{id}_map -> we want {prefix}_zone_{id}
        const prefix = mapEntity.replace(/_map$/, '');
        console.log(`[FP2 Card] ✓ Extracted prefix for device ${deviceId}: ${prefix}`);
        return prefix;
      }

      console.warn(`[FP2 Card] ✗ Could not determine prefix for device ${deviceId}`);
      return null;
    };

    // Helper to parse grid from hex bitmap string (14 rows × 4 hex chars = 56 chars)
    // Each row is represented by 4 hex characters (2 bytes), with the 14 LSBs indicating cell states
    const parseGrid = (gridString, gridName = "unknown") => {
      if (!gridString) {
        console.warn(`[FP2 Card] Grid "${gridName}" is null/undefined, using empty grid`);
        return Array(14)
          .fill(null)
          .map(() => Array(14).fill(0));
      }
      if (gridString.length !== 56) {
        console.warn(`[FP2 Card] Grid "${gridName}" has invalid length ${gridString.length} (expected 56), using empty grid`);
        return Array(14)
          .fill(null)
          .map(() => Array(14).fill(0));
      }
      const grid = [];
      for (let y = 0; y < 14; y++) {
        grid[y] = [];
        // Read 4 hex chars for this row (representing 2 bytes)
        const hexChars = gridString.substr(y * 4, 4);
        const rowBits = parseInt(hexChars, 16);

        // Extract the 14 LSBs as cell values
        for (let x = 0; x < 14; x++) {
          grid[y][x] = (rowBits >> x) & 1;
        }
      }
      console.log(`[FP2 Card] ✓ Grid "${gridName}" parsed successfully`);
      return grid;
    };

    // --- Static Zone Definitions (Global Text Sensors) ---
    // These are global zones that define the sensor's static configuration
    console.log(`[FP2 Card] Loading static grid definitions...`);
    const edgeLabelGrid = parseGrid(
      getEntityState(`${prefix}_edge_label_grid`),
      "edge_label_grid"
    );
    const entryExitGrid = parseGrid(
      getEntityState(`${prefix}_entry_exit_grid`),
      "entry_exit_grid"
    );
    const interferenceGrid = parseGrid(
      getEntityState(`${prefix}_interference_grid`),
      "interference_grid"
    );

    // Get mounting position (from config or attribute)
    const mountingPosition = this.config.mounting_position || "wall";

    // --- Detection Zones (Sub-Devices) ---
    // Each zone is registered as a sub-device (via_device_id points to main FP2 device)
    // Each zone sub-device has: map, occupancy state, and motion sensors
    console.log(`[FP2 Card] Discovering zones via device registry...`);
    const zones = [];

    const mainDeviceId = getMainDeviceId();
    if (mainDeviceId) {
      const subDevices = findZoneSubDevices(mainDeviceId);

      subDevices.forEach(({ deviceId, device }) => {
        const zonePrefix = getDeviceEntityPrefix(deviceId);
        if (!zonePrefix) return;

        // Extract zone ID from prefix (format: {prefix}_zone_{id})
        const zoneIdMatch = zonePrefix.match(/_zone_(.+)$/);
        if (!zoneIdMatch) {
          console.warn(`[FP2 Card] ✗ Could not extract zone ID from prefix: ${zonePrefix}`);
          return;
        }

        const zoneId = zoneIdMatch[1];
        console.log(`[FP2 Card] Processing zone: ${zoneId} (prefix: ${zonePrefix})`);

        const mapEntity = `${zonePrefix}_map`;
        const occupancyEntity = `${zonePrefix}_occupancy`;
        const motionEntity = `${zonePrefix}_motion`;

        const zoneMap = parseGrid(getEntityState(mapEntity), `zone_${zoneId}_map`);
        const occupancyState = getEntityState(occupancyEntity);
        const motionState = getEntityState(motionEntity);

        zones.push({
          id: zoneId,
          map: zoneMap,
          occupancy: occupancyState === "on",
          motion: motionState,
        });
        console.log(`[FP2 Card] ✓ Zone ${zoneId}: occupancy=${occupancyState}, motion=${motionState}`);
      });
    } else {
      console.warn(`[FP2 Card] ✗ Could not find main device, zones will not be loaded`);
    }

    console.log(`[FP2 Card] Total zones discovered: ${zones.length}`);

    // --- Full Location Data Sensor ---
    // Text sensor containing JSON array with all target locations
    // Format: [{ id, x, y, velocity_x, velocity_y }, ...]
    console.log(`[FP2 Card] Loading target data...`);
    const targetData = getJsonAttribute(`${prefix}_targets`);
    const targetCount = targetData ? targetData.length : 0;
    console.log(`[FP2 Card] Total targets: ${targetCount}`);

    const result = {
      edgeLabelGrid,
      entryExitGrid,
      interferenceGrid,
      mountingPosition,
      zones,
      targets: targetData || [],
    };

    console.log(`[FP2 Card] ===== Entity gathering complete =====`);
    console.log(`[FP2 Card] Summary: ${zones.length} zones, ${targetCount} targets, mounting: ${mountingPosition}`);

    return result;
  }

  renderCanvas(data) {
    // Calculate canvas dimensions
    const containerWidth = this.canvas.clientWidth || 600;
    const dpr = window.devicePixelRatio || 1;

    // Determine which cells to display
    let minX = 0,
      maxX = 13,
      minY = 0,
      maxY = 13;

    if (this.displayMode === "zoomed") {
      // Calculate bounding box of non-edge cells
      let found = false;
      minX = 13;
      maxX = 0;
      minY = 13;
      maxY = 0;

      for (let y = 0; y < 14; y++) {
        for (let x = 0; x < 14; x++) {
          if (!data.edgeLabelGrid[y][x]) {
            found = true;
            minX = Math.min(minX, x);
            maxX = Math.max(maxX, x);
            minY = Math.min(minY, y);
            maxY = Math.max(maxY, y);
          }
        }
      }

      if (!found) {
        // Fallback to full grid if no non-edge cells
        minX = 0;
        maxX = 13;
        minY = 0;
        maxY = 13;
      }
    }

    const gridWidth = maxX - minX + 1;
    const gridHeight = maxY - minY + 1;
    const cellSize = containerWidth / gridWidth;
    const canvasWidth = containerWidth;
    const canvasHeight = cellSize * gridHeight;

    // Set canvas size accounting for device pixel ratio
    this.canvas.width = canvasWidth * dpr;
    this.canvas.height = canvasHeight * dpr;
    this.canvas.style.width = `${canvasWidth}px`;
    this.canvas.style.height = `${canvasHeight}px`;

    this.ctx.scale(dpr, dpr);

    // Store for mouse interactions
    this.renderParams = {
      minX,
      maxX,
      minY,
      maxY,
      cellSize,
      canvasWidth,
      canvasHeight,
    };

    // Clear canvas
    this.ctx.clearRect(0, 0, canvasWidth, canvasHeight);

    // Draw layers in order
    this.drawBaseGrid(data, minX, maxX, minY, maxY, cellSize);
    this.drawEdgeLabels(data, minX, maxX, minY, maxY, cellSize);
    this.drawInterferenceSources(data, minX, maxX, minY, maxY, cellSize);
    this.drawEntryExitZones(data, minX, maxX, minY, maxY, cellSize);
    this.drawDetectionZones(data, minX, maxX, minY, maxY, cellSize);
    this.drawTargets(data, minX, maxX, minY, maxY, cellSize);

    if (this.showSensorPosition) {
      this.drawSensorPosition(data, minX, maxX, minY, maxY, cellSize);
    }
  }

  drawBaseGrid(data, minX, maxX, minY, maxY, cellSize) {
    this.ctx.strokeStyle = "var(--divider-color)";
    this.ctx.lineWidth = 0.5;

    for (let y = minY; y <= maxY + 1; y++) {
      const yPos = (y - minY) * cellSize;
      this.ctx.beginPath();
      this.ctx.moveTo(0, yPos);
      this.ctx.lineTo((maxX - minX + 1) * cellSize, yPos);
      this.ctx.stroke();
    }

    for (let x = minX; x <= maxX + 1; x++) {
      const xPos = (x - minX) * cellSize;
      this.ctx.beginPath();
      this.ctx.moveTo(xPos, 0);
      this.ctx.lineTo(xPos, (maxY - minY + 1) * cellSize);
      this.ctx.stroke();
    }
  }

  drawEdgeLabels(data, minX, maxX, minY, maxY, cellSize) {
    this.ctx.fillStyle = "rgba(128, 128, 128, 0.5)";

    for (let y = minY; y <= maxY; y++) {
      for (let x = minX; x <= maxX; x++) {
        if (data.edgeLabelGrid[y][x]) {
          const xPos = (x - minX) * cellSize;
          const yPos = (y - minY) * cellSize;
          this.ctx.fillRect(xPos, yPos, cellSize, cellSize);

          // Add crosshatch pattern
          this.ctx.strokeStyle = "rgba(96, 96, 96, 0.3)";
          this.ctx.lineWidth = 1;
          this.ctx.beginPath();
          this.ctx.moveTo(xPos, yPos);
          this.ctx.lineTo(xPos + cellSize, yPos + cellSize);
          this.ctx.moveTo(xPos + cellSize, yPos);
          this.ctx.lineTo(xPos, yPos + cellSize);
          this.ctx.stroke();
        }
      }
    }
  }

  drawInterferenceSources(data, minX, maxX, minY, maxY, cellSize) {
    this.ctx.fillStyle = "rgba(255, 100, 100, 0.3)";

    for (let y = minY; y <= maxY; y++) {
      for (let x = minX; x <= maxX; x++) {
        if (data.interferenceGrid[y][x]) {
          const xPos = (x - minX) * cellSize;
          const yPos = (y - minY) * cellSize;
          this.ctx.fillRect(xPos, yPos, cellSize, cellSize);
        }
      }
    }
  }

  drawEntryExitZones(data, minX, maxX, minY, maxY, cellSize) {
    this.ctx.strokeStyle = "rgba(100, 200, 100, 0.8)";
    this.ctx.lineWidth = 3;

    for (let y = minY; y <= maxY; y++) {
      for (let x = minX; x <= maxX; x++) {
        if (data.entryExitGrid[y][x]) {
          const xPos = (x - minX) * cellSize;
          const yPos = (y - minY) * cellSize;
          this.ctx.strokeRect(xPos + 2, yPos + 2, cellSize - 4, cellSize - 4);
        }
      }
    }
  }

  drawDetectionZones(data, minX, maxX, minY, maxY, cellSize) {
    data.zones.forEach((zone) => {
      // Determine zone color based on occupancy
      const baseColor = zone.occupancy
        ? "rgba(100, 150, 255, 0.6)" // Occupied: bright blue
        : "rgba(100, 150, 255, 0.2)"; // Empty: light blue

      this.ctx.fillStyle = baseColor;

      // Fill zone cells
      for (let y = minY; y <= maxY; y++) {
        for (let x = minX; x <= maxX; x++) {
          if (zone.map[y][x]) {
            const xPos = (x - minX) * cellSize;
            const yPos = (y - minY) * cellSize;
            this.ctx.fillRect(xPos, yPos, cellSize, cellSize);
          }
        }
      }

      // Draw zone border
      this.ctx.strokeStyle = zone.occupancy
        ? "rgba(50, 100, 255, 0.8)"
        : "rgba(50, 100, 255, 0.4)";
      this.ctx.lineWidth = 2;

      // Find zone bounds for label placement
      let zoneMinX = 14,
        zoneMaxX = -1,
        zoneMinY = 14,
        zoneMaxY = -1;
      for (let y = 0; y < 14; y++) {
        for (let x = 0; x < 14; x++) {
          if (zone.map[y][x]) {
            zoneMinX = Math.min(zoneMinX, x);
            zoneMaxX = Math.max(zoneMaxX, x);
            zoneMinY = Math.min(zoneMinY, y);
            zoneMaxY = Math.max(zoneMaxY, y);
          }
        }
      }

      // Draw border around zone
      if (zoneMinX <= zoneMaxX) {
        for (let y = minY; y <= maxY; y++) {
          for (let x = minX; x <= maxX; x++) {
            if (zone.map[y][x]) {
              const xPos = (x - minX) * cellSize;
              const yPos = (y - minY) * cellSize;
              this.ctx.strokeRect(
                xPos + 1,
                yPos + 1,
                cellSize - 2,
                cellSize - 2,
              );
            }
          }
        }

        // Draw zone label if enabled and zone is visible
        if (
          this.showZoneLabels &&
          zoneMinX >= minX &&
          zoneMaxX <= maxX &&
          zoneMinY >= minY &&
          zoneMaxY <= maxY
        ) {
          const labelX = (zoneMinX + zoneMaxX) / 2;
          const labelY = (zoneMinY + zoneMaxY) / 2;
          const xPos = (labelX - minX) * cellSize + cellSize / 2;
          const yPos = (labelY - minY) * cellSize + cellSize / 2;

          this.ctx.fillStyle = "rgba(255, 255, 255, 0.9)";
          this.ctx.strokeStyle = "rgba(50, 100, 255, 0.8)";
          this.ctx.lineWidth = 2;

          const labelText = `Z${zone.id}`;
          this.ctx.font = `bold ${Math.min(cellSize * 0.6, 16)}px sans-serif`;
          this.ctx.textAlign = "center";
          this.ctx.textBaseline = "middle";

          const metrics = this.ctx.measureText(labelText);
          const padding = 4;
          this.ctx.fillRect(
            xPos - metrics.width / 2 - padding,
            yPos - cellSize * 0.3 - padding,
            metrics.width + padding * 2,
            cellSize * 0.6 + padding * 2,
          );

          this.ctx.fillStyle = "rgba(50, 100, 255, 1)";
          this.ctx.fillText(labelText, xPos, yPos);
        }
      }
    });
  }

  drawTargets(data, minX, maxX, minY, maxY, cellSize) {
    if (!data.targets || !Array.isArray(data.targets)) return;

    data.targets.forEach((target) => {
      let gridX, gridY;

      // Convert raw coordinates to grid space based on mounting mode
      if (data.mountingPosition === "left_upper_corner" || data.mountingPosition === "right_upper_corner") {
        // Corner mounting modes: 14x14 grid, 7m x 7m area
        // Raw coordinates: X in [-400, +400], Y in [0, 800]
        // X = +400: Left edge, X = -400: Right edge
        // Y = 0: Top edge (closest to sensor), Y = 800: Bottom edge (farthest from sensor)
        // Conversion: Grid_X = (X + 400) / 800.0 * 14.0, Grid_Y = Y / 800.0 * 14.0
        gridX = (target.x + 400) / 800.0 * 14.0;
        gridY = target.y / 800.0 * 14.0;
      } else {
        // Wall mounting mode - TODO: coordinate conversion not yet verified
        // Stubbing with basic conversion for now
        console.warn(`[FP2 Card] Wall mounting mode coordinate conversion is not yet implemented, using placeholder`);
        gridX = target.x * 0.01; // Placeholder
        gridY = target.y * 0.01; // Placeholder
      }

      const x = gridX;
      const y = gridY;

      // Check if target is in visible range
      if (x < minX || x > maxX || y < minY || y > maxY) return;

      const xPos = (x - minX) * cellSize + cellSize / 2;
      const yPos = (y - minY) * cellSize + cellSize / 2;
      const radius = Math.min(cellSize * 0.3, 15);

      // Draw target circle
      this.ctx.fillStyle = "rgba(255, 200, 0, 0.8)";
      this.ctx.strokeStyle = "rgba(255, 150, 0, 1)";
      this.ctx.lineWidth = 2;

      this.ctx.beginPath();
      this.ctx.arc(xPos, yPos, radius, 0, Math.PI * 2);
      this.ctx.fill();
      this.ctx.stroke();

      // Draw target ID
      this.ctx.fillStyle = "rgba(0, 0, 0, 0.8)";
      this.ctx.font = `bold ${Math.min(cellSize * 0.4, 12)}px sans-serif`;
      this.ctx.textAlign = "center";
      this.ctx.textBaseline = "middle";
      this.ctx.fillText(target.id || "?", xPos, yPos);

      // Draw velocity vector if available
      if (target.velocity_x !== undefined && target.velocity_y !== undefined) {
        let vx, vy;

        // Convert velocity using the same coordinate system as position
        if (data.mountingPosition === "left_upper_corner" || data.mountingPosition === "right_upper_corner") {
          // Velocity scaling: same as position, but without the bias offset
          vx = target.velocity_x / 800.0 * 14.0;
          vy = target.velocity_y / 800.0 * 14.0;
        } else {
          // Wall mounting mode - placeholder
          vx = target.velocity_x * 0.01;
          vy = target.velocity_y * 0.01;
        }

        const magnitude = Math.sqrt(vx * vx + vy * vy);

        if (magnitude > 0.1) {
          const arrowLength = Math.min(
            magnitude * cellSize * 2,
            cellSize * 1.5,
          );
          const endX = xPos + (vx / magnitude) * arrowLength;
          const endY = yPos + (vy / magnitude) * arrowLength;

          this.ctx.strokeStyle = "rgba(255, 150, 0, 0.8)";
          this.ctx.lineWidth = 2;
          this.ctx.beginPath();
          this.ctx.moveTo(xPos, yPos);
          this.ctx.lineTo(endX, endY);
          this.ctx.stroke();

          // Draw arrowhead
          const angle = Math.atan2(vy, vx);
          const arrowSize = 8;
          this.ctx.beginPath();
          this.ctx.moveTo(endX, endY);
          this.ctx.lineTo(
            endX - arrowSize * Math.cos(angle - Math.PI / 6),
            endY - arrowSize * Math.sin(angle - Math.PI / 6),
          );
          this.ctx.moveTo(endX, endY);
          this.ctx.lineTo(
            endX - arrowSize * Math.cos(angle + Math.PI / 6),
            endY - arrowSize * Math.sin(angle + Math.PI / 6),
          );
          this.ctx.stroke();
        }
      }
    });
  }

  drawSensorPosition(data, minX, maxX, minY, maxY, cellSize) {
    let sensorX, sensorY;

    switch (data.mountingPosition) {
      case "left_upper_corner":
        // Top left corner of top-left cell (0,0)
        sensorX = 0;
        sensorY = 0;
        break;
      case "right_upper_corner":
        // Top right corner of top-right cell (13,0)
        sensorX = 14; // Right edge of cell 13
        sensorY = 0;
        break;
      case "wall":
      default:
        // Top edge between middle cells (between cells 6 and 7)
        sensorX = 7; // Grid line between cells 6 and 7
        sensorY = 0;
        break;
    }

    // Only draw if sensor is in visible range
    if (
      sensorX < minX - 1 ||
      sensorX > maxX + 1 ||
      sensorY < minY - 1 ||
      sensorY > maxY + 1
    ) {
      return;
    }

    const xPos = (sensorX - minX) * cellSize;
    const yPos = (sensorY - minY) * cellSize;
    const size = cellSize * 0.4;

    // Draw sensor icon (radar waves)
    this.ctx.strokeStyle = "rgba(255, 100, 100, 0.7)";
    this.ctx.lineWidth = 2;

    for (let i = 0; i < 3; i++) {
      this.ctx.beginPath();
      this.ctx.arc(xPos, yPos, size + i * 8, 0, Math.PI);
      this.ctx.stroke();
    }

    // Draw sensor dot
    this.ctx.fillStyle = "rgba(255, 50, 50, 0.9)";
    this.ctx.beginPath();
    this.ctx.arc(xPos, yPos, 4, 0, Math.PI * 2);
    this.ctx.fill();
  }

  updateInfoPanel(data) {
    const activeZones = data.zones.filter((z) => z.occupancy).length;
    const totalZones = data.zones.length;
    const targetCount = data.targets ? data.targets.length : 0;

    this.infoPanel.innerHTML = `
      <strong>Display Mode:</strong> ${this.displayMode === "full" ? "Full Grid (14×14)" : "Zoomed View"} |
      <strong>Zones:</strong> ${activeZones}/${totalZones} occupied |
      <strong>Targets:</strong> ${targetCount}
    `;
  }

  handleCanvasClick(e) {
    if (!this.renderParams) return;

    const rect = this.canvas.getBoundingClientRect();
    const x = e.clientX - rect.left;
    const y = e.clientY - rect.top;

    const { minX, minY, cellSize } = this.renderParams;
    const gridX = Math.floor(x / cellSize) + minX;
    const gridY = Math.floor(y / cellSize) + minY;

    console.log(`Clicked cell: (${gridX}, ${gridY})`);

    // You can add more interactive features here
    // e.g., show detailed info in a popup
  }

  handleCanvasHover(e) {
    // Implement hover effects if needed
  }

  getCardSize() {
    return 4;
  }

  getGridOptions() {
    return {
      rows: 4,
      columns: 6,
      min_rows: 3,
      max_rows: 6,
    };
  }
}

customElements.define("aqara-fp2-card", AqaraFP2Card);

window.customCards = window.customCards || [];
window.customCards.push({
  type: "aqara-fp2-card",
  name: "Aqara FP2 Presence Sensor Card",
  description:
    "Visualizes Aqara FP2 presence sensor data with zones and target tracking",
  preview: true,
});
