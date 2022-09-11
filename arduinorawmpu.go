package spotpuppy

// For the arduino sketch, refer to this repo: github.com/JoshPattman/arduino-raw-mpu5060

import (
	"encoding/json"
	"time"

	"github.com/tarm/serial"
)

type rotationPacket struct {
	gyroX, gyroY, gyroZ    float64
	accelX, accelY, accelZ float64
}

type RawArduinoRotationSensor struct {
	Port               *serial.Port  `json:"-"`
	IsReady            bool          `json:"-"`
	PortName           string        `json:"port_name"`
	Axes               *AxesRemapper `json:"axes_remap"`
	ReverseGyroUp      bool          `json:"rev_gyro_up"`
	ReverseGyroLeft    bool          `json:"rev_gyro_left"`
	ReverseGyroForward bool          `json:"rev_gyro_forward"`
	// Maximum number of deg/s the accelerometer can move the rotation
	AccSpeed    float64 `json:"acc_speed"`
	calibration rotationPacket
	cachedRot   Quat
	stopFlag    bool
	stoppedFlag bool
}

// Creates a new sensor instance. Does not connect to the arduino yet, that is done from Setup()
func NewRawArduinoRotationSensor() *RawArduinoRotationSensor {
	return &RawArduinoRotationSensor{
		IsReady:   false,
		PortName:  "/dev/ttyUSB0",
		Axes:      NewAxesRemapper(Forward, Left, Up),
		cachedRot: QuatIdentity,
		AccSpeed:  180,
	}
}

// Sets up and connects to the arduino
func (a *RawArduinoRotationSensor) Setup() {
	c := &serial.Config{Name: a.PortName, Baud: 115200}
	s, err := serial.OpenPort(c)
	if err != nil {
		panic("Failed to connect to arduino on port " + a.PortName)
	}
	s.Flush()
	a.Port = s
	go a.updateInBackground()
	a.IsReady = true
}

// Restarts the updating in background thread
func (a *RawArduinoRotationSensor) Restart() {
	// Wait for update thread to stop
	a.stoppedFlag = false
	a.stopFlag = true
	for !a.stoppedFlag {
		time.Sleep(time.Second / 10)
	}
	// Restart update in background
	go a.updateInBackground()
}

// Calibrates the sensors accelerometer and gyro. Ensure the sensor is very flat for this
func (a *RawArduinoRotationSensor) Calibrate() {
	if !a.IsReady {
		panic("Rotation sensor wasn't ready")
	}
	// Wait for update thread to stop
	a.stoppedFlag = false
	a.stopFlag = true
	for !a.stoppedFlag {
		time.Sleep(time.Second / 10)
	}

	// Read the rotation packet
	d := rotationPacket{}
	for i := 0; i < 100; i++ {
		d1 := a.parseNextPacket()
		d.accelX += d1.accelX
		// We add 0.5 here as you should take away the maximum force in the direction of u = -1*-0.5 = +0.5
		d.accelY += d1.accelY + 0.5
		d.accelZ += d1.accelZ
		d.gyroX += d1.gyroX
		d.gyroY += d1.gyroY
		d.gyroZ += d1.gyroZ
	}
	d.accelX /= 100
	d.accelY /= 100
	d.accelZ /= 100
	d.gyroX /= 100
	d.gyroY /= 100
	d.gyroZ /= 100
	a.calibration = d

	// Restart update in background
	go a.updateInBackground()
}

// Returns the last measured rotation of the sensor. Non blocking
func (a *RawArduinoRotationSensor) GetQuaternion() Quat {
	return a.cachedRot
}

// This runs constantly in the background so that the update loop always gets the most up to dat info without having to wait
func (a *RawArduinoRotationSensor) updateInBackground() {
	lastUpdate := time.Now()
	// Clean out any old data sat in the port
	a.Port.Flush()
	a.cachedRot = QuatIdentity
	for {
		// Check if we need to stop
		if a.stopFlag {
			a.stopFlag = false
			a.stoppedFlag = true
			return
		}

		// Time managment
		thisUpdate := time.Now()
		dt := thisUpdate.Sub(lastUpdate)
		lastUpdate = thisUpdate

		// Read the serial
		p := a.parseNextPacket()

		// Remove the calibration offsets
		p.accelX -= a.calibration.accelX
		p.accelY -= a.calibration.accelY
		p.accelZ -= a.calibration.accelZ
		p.gyroX -= a.calibration.gyroX
		p.gyroY -= a.calibration.gyroY
		p.gyroZ -= a.calibration.gyroZ

		// Copy the current cached rotation so we can do calculations on it
		orientation := a.cachedRot

		// Calculate update quaternion based on gyro
		rawGyroVec := NewVector3(p.gyroX, p.gyroY, p.gyroZ)
		gyroAngle := rawGyroVec.Len()
		if gyroAngle != 0 {
			gyroAxis := rawGyroVec.Unit()
			q := NewQuatAngleAxis(gyroAxis, -gyroAngle*dt.Seconds())
			orientation = orientation.RotateByLocal(q)
		}

		// Calculate the vector relative to our current orientation that points at true Up (accel)
		accelUp := NewVector3(p.accelX, p.accelY, p.accelZ).Unit().Rotated(orientation)
		orientationUp := Up

		// Calculate the quaternion we need to rotate by to move towards our true rotation,  then apply it
		q := NewQuatFromTo(accelUp, orientationUp)
		angleMult := accelUp.AngleTo(orientationUp) / 180.0
		orientation = orientation.RotateByGlobal(NewQuatAngleAxis(NewVector3(q.X, q.Y, q.Z), a.AccSpeed*dt.Seconds()*angleMult))

		// Copy our new rotation back to the cachedRot
		a.cachedRot = orientation
	}
}

// Waits for then reads and parses the next packet sent by arduino. Then converts the packet to spotpuppy coordinate system and reverses gyros is need be
func (a *RawArduinoRotationSensor) parseNextPacket() rotationPacket {
	for {
		buf := make([]byte, 1)
		msg := make([]byte, 0)
		for {
			_, err := a.Port.Read(buf)
			if err != nil {
				panic(err)
			}
			if buf[0] == '[' {
				msg = append(msg, buf[0])
				break
			}
		}
		for {
			_, err := a.Port.Read(buf)
			if err != nil {
				panic(err)
			}
			msg = append(msg, buf[0])
			if buf[0] == ']' {
				break
			}
		}
		values := make([]float64, 6)
		e := json.Unmarshal(msg, &values)
		if e == nil && len(values) == 6 {
			gyroData := a.Axes.Remap(NewVector3(values[0], values[1], values[2]))
			accData := a.Axes.Remap(NewVector3(values[3], values[4], values[5]))
			if a.ReverseGyroForward {
				gyroData.X *= -1
			}
			if a.ReverseGyroUp {
				gyroData.Y *= -1
			}
			if a.ReverseGyroLeft {
				gyroData.Z *= -1
			}
			return rotationPacket{
				gyroX:  gyroData.X,
				gyroY:  gyroData.Y,
				gyroZ:  gyroData.Z,
				accelX: accData.X,
				accelY: accData.Y,
				accelZ: accData.Z,
			}
		}
	}
}
