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
	Port        *serial.Port `json:"-"`
	IsReady     bool         `json:"-"`
	PortName    string       `json:"port_name"`
	Axes        AxesRemap    `json:"axes_remap"`
	parsedAxes  []Axes
	calibration rotationPacket
	cachedRot   Quat
	stopFlag    bool
	stoppedFlag bool
}

func NewRawArduinoRotationSensor() *RawArduinoRotationSensor {
	return &RawArduinoRotationSensor{
		IsReady:  false,
		PortName: "/dev/ttyUSB0",
		Axes: AxesRemap{
			X: "x",
			Y: "y",
			Z: "z",
		},
		cachedRot: QuatIdentity,
	}
}

func (a *RawArduinoRotationSensor) Setup() {
	c := &serial.Config{Name: a.PortName, Baud: 115200}
	s, err := serial.OpenPort(c)
	if err != nil {
		panic("Failed to connect to arduino on port " + a.PortName)
	}
	s.Flush()
	p := ParseAxesRemap(a.Axes)
	a.parsedAxes = p
	a.Port = s
	go a.updateInBackground()
	a.IsReady = true
}

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
		d1 := a.readNextPacket()
		d.accelX += d1.accelX
		d.accelY += d1.accelY
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

func (a *RawArduinoRotationSensor) GetQuaternion() Quat {
	q := a.cachedRot
	return q.RemapAxesFrom(a.parsedAxes[0], a.parsedAxes[1], a.parsedAxes[2])
}

// Waits for then reads the next packet sent bu arduino
func (a *RawArduinoRotationSensor) readNextPacket() rotationPacket {
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
			return rotationPacket{
				gyroX:  values[0],
				gyroY:  values[1],
				gyroZ:  values[2],
				accelX: values[3],
				accelY: values[4],
				accelZ: values[5],
			}
		}
	}
}

// This runs constantly in the background so that the update loop always gets the most up to dat info without having to wait
func (a *RawArduinoRotationSensor) updateInBackground() {
	lastUpdate := time.Now()
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
		p := a.readNextPacket()

		// Remove the calibration offsets
		p.accelX -= a.calibration.accelX
		p.accelY -= a.calibration.accelY
		p.accelZ -= a.calibration.accelZ
		p.gyroX -= a.calibration.gyroX
		p.gyroY -= a.calibration.gyroY
		p.gyroZ -= a.calibration.gyroZ

		// Copy the current cached rotation so we can do calculations on it
		orientation := a.cachedRot

		// Calculate and update quaternion based on gyro
		rawGyroVec := NewVector3(p.gyroX, p.gyroY, p.gyroZ)
		gyroAngle := rawGyroVec.Len()
		if gyroAngle != 0 {
			gyroAxis := rawGyroVec.Unit()
			q := NewQuatAngleAxis(gyroAxis, gyroAngle*dt.Seconds())
			orientation = orientation.RotateByLocal(q)
		}
		a.cachedRot = orientation
	}
}
