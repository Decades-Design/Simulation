let cruiseAlt = { min: 30000, max: 41000 };

const MTOW = 73900; // kg
const maxThrust = 118000 * 2; // Newtons (2x engines)
const wingArea = 122.6; // m^2
const frontalArea = 30; // m^2
const Cd0 = 0.1; // Zero-lift drag coefficient for the aircraft
const inducedDragFactor = Math.PI * 9.5 * 0.78; // Induced drag factor for the aircraft

const rotateSpeed = 150;
const initialClimbSpeed = 200; // Initial climb speed in knots
const lowClimbSpeed = 250; // Low climb speed in knots
const highClimbSpeed = 290; // High climb speed in knots
const highDescentSpeed = 300; // High descent speed in knots
const lowDescentSpeed = 250; // Low descent speed in knots
const landingSpeed = 137; // Landing speed in knots

const NM_TO_KM = 1.852; // Nautical Miles to Kilometers conversion factor.
const KNOTS_TO_KPS = 0.000514444; // Knots (nautical miles per hour) to Kilometers Per Second.
const FEET_TO_KM = 0.0003048; // Feet to Kilometers.
const KNOTS_TO_MPS = 0.51444; // Knots to Meters Per Second
const METERS_TO_FEET = 3.28084; // Meters to Feet conversion factor.

const ISA_TEMP_C = 15; // Standard temperature at sea level (Celsius)
const ISA_LAPSE_RATE_C = 1.98; // Temp drop per 1000 ft (Celsius)
const ISA_PRESSURE_HPA = 1013.25; // Standard pressure at sea level (hPa)
const seaLevelDensity = 1.225; // Standard air density at sea level (kg/m^3)

let aircraftList = [];

class Aircraft {
  constructor() {
    this.airportAlt = 0;
    this.cruiseAlt =
      cruiseAlt.min + Math.random() * (cruiseAlt.max - cruiseAlt.min);

    this.mass = 70000;
    this.maxThrust = maxThrust;
    this.wingArea = wingArea;
    this.frontalArea = frontalArea;

    this.thrust = 0;
    this.drag = 0;
    this.lift = 0;
    this.weight = this.mass * 9.81; // Weight = mass * gravity
    this.liftCoefficient = 0;
    this.Cd0 = Cd0;
    this.inducedDragFactor = inducedDragFactor;

    this.rotateSpeed = rotateSpeed;
    this.initialClimbSpeed = initialClimbSpeed;
    this.lowClimbSpeed = lowClimbSpeed;
    this.highClimbSpeed = highClimbSpeed;
    this.highDescentSpeed = highDescentSpeed;
    this.lowDescentSpeed = lowDescentSpeed;
    this.landingSpeed = landingSpeed;

    this.altitude = 0;
    this.targetAlt = 0;
    this.verticalSpeed = 0;

    this.heading = 360;
    this.targetHdg = this.heading;

    this.indicatedAirspeed = 0;
    this.targetSpd = 0;
    this.trueAirspeed = this.rotateSpeed;
    this.groundSpeed = 0;
    this.acceleration = 0;

    this.pitch = 10; // Pitch angle in degrees (start with a slight nose-up attitude)
    this.pitchRate = 0; // Pitch rate in degrees per second
    this.angleOfAttack = 0; // Angle of Attack (AoA) in degrees

    this.pitchPid = {
      prevError: 0,
      integral: 0,
    };

    this.history = {
        time: [],
        pitch: [],
        pitchRate: [],
        angleOfAttack: [],
        altitude: [],
        verticalSpeed: [],
        indicatedAirspeed: [],
        liftCoefficient: [],
        targetSpeed: [],
        speedError: [],
        pidIntegral: [],
        pidDerivative: []
    };
  }

  updateClimbPhase(deltaTime) {
    // --- 1. ATMOSPHERIC CALCULATIONS (No changes) ---
    const tempC = ISA_TEMP_C - (this.altitude / 1000) * ISA_LAPSE_RATE_C;
    const tempK = tempC + 273.15;
    const pressureHpa =
      ISA_PRESSURE_HPA *
      Math.pow(1 - (0.0065 * (this.altitude / METERS_TO_FEET)) / 288.15, 5.255);
    const airDensity = (pressureHpa * 100) / (287.05 * tempK);
    const trueAirspeedMPS = this.trueAirspeed * KNOTS_TO_MPS;

    // --- 2. TARGET SPEED & THRUST LOGIC (No changes) ---
    if (this.altitude < this.airportAlt + 1500) {
      this.thrust = this.maxThrust;
      this.targetSpd = this.initialClimbSpeed;
    } else {
      this.thrust = this.maxThrust * 0.85; // Using the more realistic 85% climb thrust
      this.targetSpd = this.lowClimbSpeed;
    }

    // ===================================================================
    // NEW PITCH-BASED FLIGHT DYNAMICS
    // ===================================================================
    
    // --- 3. PITCH CONTROL (AUTOPILOT) ---
    // The PID controller now adjusts pitch rate to achieve the target airspeed.
    const speedError = this.targetSpd - this.indicatedAirspeed;
    const Kp = -10; 
    const Ki = -0.1;
    const Kd = -50;

    // Simple PID function for pitch with anti-windup
    this.pitchPid.integral += speedError * deltaTime;

    const derivative = (speedError - this.pitchPid.prevError) / deltaTime;
    this.pitchPid.prevError = speedError;

    // The output is a desired PITCH RATE. If speed is low, pitch down (negative rate).
    const desiredPitchRate =  (Kp * speedError + Ki * this.pitchPid.integral + Kd * derivative);
    
    // --- 4. ROTATIONAL KINEMATICS (Simplified) ---
    // We directly adjust the pitch rate towards the rate desired by the PID controller.
    // This simulates the aircraft's control response without complex moment calculations.
    const pitchResponseFactor = 0.0001; // How quickly the aircraft responds to pitch commands (higher is faster)

    // The change in pitchRate is proportional to the difference between the desired and current rates.
    const pitchRateError = desiredPitchRate - this.pitchRate;
    //this.pitchRate += pitchRateError * pitchResponseFactor * deltaTime;
    this.pitchRate = desiredPitchRate * pitchResponseFactor; // Directly set pitchRate for more immediate response

    // Update pitch angle from the new pitch rate
    this.pitch += this.pitchRate * deltaTime;
    
    // --- 5. FLIGHT PATH & ANGLE OF ATTACK ---
    // Calculate the flight path angle (gamma), which is the direction of travel
    // Convert this.verticalSpeed (ft/min) to m/s for calculations
    const verticalSpeedMPS = this.verticalSpeed * 0.00508; // 1 ft/min = 0.00508 m/s
    const flightPathAngle = Math.atan2(verticalSpeedMPS, trueAirspeedMPS) * (180 / Math.PI); // in degrees

    // Angle of Attack is the difference between where the nose is pointing (pitch) and where the aircraft is going (flight path)
    this.angleOfAttack = this.pitch - flightPathAngle;
    
    // --- 6. AOA to CL CONVERSION (PLACEHOLDER) ---
    // This is a crucial new step. For now, a simple linear approximation is used.
    // A typical Cl-alpha slope for a subsonic airfoil is ~0.1 per degree.
    const clAlphaSlope = 0.14;
    const zeroLiftAoA = -5; // The AoA at which the wing produces zero lift
    this.liftCoefficient = clAlphaSlope * (this.angleOfAttack - zeroLiftAoA);
    //this.liftCoefficient = Math.min(Math.max(this.liftCoefficient, -0.5), 1.6); // Clamp to realistic limits

    // --- 7. FORCE CALCULATIONS (Updated) ---
    this.lift = 0.5 * airDensity * Math.pow(trueAirspeedMPS, 2) * this.wingArea * this.liftCoefficient;
    this.drag = 0.5 * airDensity * Math.pow(trueAirspeedMPS, 2) * this.wingArea * (this.Cd0 + Math.pow(this.liftCoefficient, 2) / this.inducedDragFactor);

    // --- 8. STATE UPDATE (Updated) ---
    // Forces are now resolved along the flight path, not just vertically/horizontally
    const flightDirectionNetForce = this.thrust * Math.cos(this.angleOfAttack * (Math.PI / 180)) - this.drag - this.weight * Math.sin(flightPathAngle * (Math.PI / 180));
    this.acceleration = flightDirectionNetForce / this.mass; // This is now acceleration ALONG the flight path

    // Update TAS from the new acceleration
    this.trueAirspeed += (this.acceleration / KNOTS_TO_MPS) * deltaTime;
    this.indicatedAirspeed = this.trueAirspeed * Math.sqrt(airDensity / seaLevelDensity);

    // Update vertical speed based on lift, weight, and flight path
    const verticalForce = this.lift * Math.cos(flightPathAngle * (Math.PI / 180)) + this.thrust * Math.sin(this.pitch * (Math.PI / 180)) - this.drag * Math.sin(flightPathAngle * (Math.PI / 180)) - this.weight;
    const verticalAcceleration = verticalForce / this.mass;
    const newVerticalSpeedMPS = verticalSpeedMPS + verticalAcceleration * deltaTime;
    // If on the ground (altitude at or below airport altitude), vertical speed cannot be negative
    if (this.altitude <= this.airportAlt) {
      this.verticalSpeed = Math.max(0, newVerticalSpeedMPS / 0.00508); // Convert m/s to ft/min
      this.altitude += this.verticalSpeed * deltaTime; // Stay on the ground
    } else {
      this.verticalSpeed = newVerticalSpeedMPS / 0.00508; // Convert m/s to ft/min
      this.altitude += newVerticalSpeedMPS * deltaTime / FEET_TO_KM / 1000; // Convert m to ft, then to altitude
      // Prevent sinking below airport altitude due to numerical errors
      if (this.altitude < this.airportAlt) {
        this.altitude = this.airportAlt;
        this.verticalSpeed = 0;
      }
    }

    // const horizontalForce = this.thrust * Math.cos(this.pitch * (Math.PI / 180)) - this.drag * Math.cos(flightPathAngle * (Math.PI / 180)) - this.lift * Math.sin(flightPathAngle * (Math.PI / 180));
    // const horizontalAcceleration = horizontalForce / this.mass;
    // const newHorizontalSpeedMPS = this.groundSpeed + horizontalAcceleration * deltaTime;
    // this.groundSpeed = newHorizontalSpeedMPS / KNOTS_TO_MPS;

    console.log(`Altitude: ${this.altitude.toFixed(2)} ft`);
    console.log(`Vertical Speed: ${this.verticalSpeed.toFixed(2)} ft/min`);
    console.log(`Target Speed: ${this.targetSpd.toFixed(2)} knots`);
    console.log(`Indicated Airspeed: ${this.indicatedAirspeed.toFixed(2)} knots`);
    console.log(`Pitch: ${this.pitch.toFixed(2)}°`);

    // --- 9. DATA RECORDING ---
    // Make sure to add the new properties to your history object in the constructor if you want to record them
    this.history.time.push(simulationTime.toFixed(3));
    this.history.altitude.push(this.altitude.toFixed(3));
    this.history.pitch.push(this.pitch.toFixed(3));
    this.history.pitchRate.push(this.pitchRate.toFixed(3));
    this.history.angleOfAttack.push(this.angleOfAttack.toFixed(3));
    this.history.verticalSpeed.push(this.verticalSpeed.toFixed(3));
    this.history.indicatedAirspeed.push(this.indicatedAirspeed.toFixed(3));
    this.history.liftCoefficient.push(this.liftCoefficient.toFixed(3));
    this.history.targetSpeed.push(this.targetSpd.toFixed(3));
    this.history.speedError.push(Kp * speedError.toFixed(3));
    this.history.pidIntegral.push(Ki * this.pitchPid.integral.toFixed(3));
    this.history.pidDerivative.push(Kd * derivative.toFixed(3));

    if (this.history.pitch) this.history.pitch.push(this.pitch.toFixed(3));
    if (this.history.aoa) this.history.aoa.push(this.angleOfAttack.toFixed(3));
}

  PIDController(targetError, Kp, Ki, Kd, deltaTime) {
    this.pitchPid.integral += targetError * deltaTime;
    const derivative = (targetError - this.pitchPid.prevError) / deltaTime;
    const pidOutput = Kp * targetError + Ki * this.pitchPid.integral + Kd * derivative;

    this.pitchPid.prevError = targetError;

    return pidOutput;
  }
}

let lastUpdateTime = 0; // The timestamp of the last frame update, used to calculate delta time.
let simulationTime = 0; // Add this variable

function simulationLoop(currentTime) {
  // On the very first frame, just set the time and skip the update
  if (lastUpdateTime === 0) {
    lastUpdateTime = currentTime;
    requestAnimationFrame(simulationLoop);
    return; // Exit the function here for the first frame
  }

  // This code will now only run from the second frame onwards
  const deltaTimeMs = currentTime - lastUpdateTime;
  lastUpdateTime = currentTime;

  // We add a small guard to prevent division by zero if the tab is inactive
  if (deltaTimeMs > 0) {
    const deltaTimeSec = deltaTimeMs / 1000;
    simulationTime += deltaTimeSec; // Increment the simulation time
    aircraftList.forEach(plane => plane.updateClimbPhase(deltaTimeSec));
  }

  requestAnimationFrame(simulationLoop);
}

aircraftList.push(new Aircraft());

requestAnimationFrame(simulationLoop);
