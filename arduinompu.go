package spotpuppy

import (
	"strconv"
	"strings"

	"github.com/tarm/serial"
)

const DefaultPiUsbPort = "/dev/ttyUSB0"

type ArduinoRotationSensor struct {
	Port       *serial.Port `json:"-"`
	IsReady    bool         `json:"-"`
	PortName   string       `json:"port_name"`
	Axes       AxesRemap    `json:"axes_remap"`
	parsedAxes []Axes
}

func (a *ArduinoRotationSensor) GetRollPitch() (float64, float64) {
	a.Port.Write([]byte{'d'})
	waitForByte('D', a.Port)
	msg := string(readUpTo(';', a.Port))
	parts := strings.Split(msg, ",")
	if len(parts) != 2 {
		panic("Invalid message received from arduino (not in format 'D%f,%f;')")
	}
	r, err := strconv.ParseFloat(parts[0], 64)
	if err != nil {
		panic("Invalid message received from arduino (could not parse float " + parts[0] + ")")
	}
	p, err := strconv.ParseFloat(parts[1], 64)
	if err != nil {
		panic("Invalid message received from arduino (could not parse float " + parts[1] + ")")
	}
	return r, p
}

func (a *ArduinoRotationSensor) GetQuaternion() Quat {
	a.Port.Write([]byte{'q'})
	waitForByte('Q', a.Port)
	msg := string(readUpTo(';', a.Port))
	parts := strings.Split(msg, ",")
	if len(parts) != 4 {
		panic("Invalid message received from arduino (not in format 'Q%f,%f,%f,%f;')")
	}
	w, err := strconv.ParseFloat(parts[0], 64)
	if err != nil {
		panic("Invalid message received from arduino (could not parse float " + parts[0] + ")")
	}
	x, err := strconv.ParseFloat(parts[1], 64)
	if err != nil {
		panic("Invalid message received from arduino (could not parse float " + parts[1] + ")")
	}
	y, err := strconv.ParseFloat(parts[2], 64)
	if err != nil {
		panic("Invalid message received from arduino (could not parse float " + parts[2] + ")")
	}
	z, err := strconv.ParseFloat(parts[3], 64)
	if err != nil {
		panic("Invalid message received from arduino (could not parse float " + parts[3] + ")")
	}
	return NewQuat(w, x, y, z).RemapAxesFrom(a.parsedAxes[0], a.parsedAxes[1], a.parsedAxes[2])
}

func NewArduinoRotationSensor() *ArduinoRotationSensor {
	return &ArduinoRotationSensor{
		IsReady: false,
	}
}

func (a *ArduinoRotationSensor) Setup() {
	c := &serial.Config{Name: a.PortName, Baud: 115200}
	s, err := serial.OpenPort(c)
	if err != nil {
		panic("Failed to connect to arduino on port " + a.PortName)
	}
	s.Write([]byte{'r'})
	waitForByte('R', s)
	p := ParseAxesRemap(a.Axes)
	a.parsedAxes = p
	a.Port = s
	a.IsReady = true
}

func (a *ArduinoRotationSensor) Calibrate() {
	if !a.IsReady {
		panic("Rotation sensor wasn't ready")
	}
	a.IsReady = false
	a.Port.Write([]byte{'c'})
	waitForByte('C', a.Port)
	a.IsReady = true
}

func waitForByte(b byte, port *serial.Port) {
	for {
		buf := make([]byte, 1)
		port.Read(buf)
		if buf[0] == b {
			return
		}
	}
}

func readUpTo(b byte, port *serial.Port) []byte {
	msg := make([]byte, 0)
	c := byte('*')
	for c != b {
		buf := make([]byte, 1)
		port.Read(buf)
		c = buf[0]
		if c != b {
			msg = append(msg, c)
		}
	}
	return msg
}
