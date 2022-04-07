package arduinompu

import (
	"github.com/tarm/serial"
	"strconv"
	"strings"
)

const DefaultPiUsbPort = "/dev/ttyUSB0"

type ArduinoMpu struct {
	Port     *serial.Port
	IsReady  bool
	InverseR bool
	InverseP bool
	FlipRP   bool
}

func (a *ArduinoMpu) GetRollPitch() (float64, float64) {
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
	if a.FlipRP {
		t := r
		r = p
		p = t
	}
	if a.InverseR {
		r = -r
	}
	if a.InverseP {
		p = -p
	}
	return r, p
}

func NewArduinoMpu(portName string) *ArduinoMpu {
	c := &serial.Config{Name: portName, Baud: 115200}
	s, err := serial.OpenPort(c)
	if err != nil {
		panic("Failed to connect to arduino on port " + portName)
	}
	s.Write([]byte{'r'})
	waitForByte('R', s)
	return &ArduinoMpu{
		Port:    s,
		IsReady: true,
	}
}

func (a *ArduinoMpu) Calibrate() {
	if !a.IsReady {
		panic("Rotation sensor wasn't ready")
	}
	a.IsReady = false
	a.Port.Write([]byte{'c'})
	waitForByte('C', a.Port)
	a.IsReady = true
}

func waitForByte(b byte, port *serial.Port) {
	for true {
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
