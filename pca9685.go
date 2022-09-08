package spotpuppy

import (
	"github.com/googolgl/go-i2c"
	"github.com/googolgl/go-pca9685"
)

type PCAMotorController struct {
	ServoOptions *pca9685.ServOptions `json:"servo-options"`
	Mapping      map[string]int       `json:"mapping"`
	servos       map[string]*pca9685.Servo
	pca          *pca9685.PCA9685
}

func (d *PCAMotorController) SetMotor(s string, a float64) {
	d.servos[s].Angle(int(a + float64(d.ServoOptions.AcRange)/2))
}

func (d *PCAMotorController) CreateMotorMapping(names []string) {
	d.Mapping = make(map[string]int)
	for _, n := range names {
		d.Mapping[n] = -1
	}
}

func (d *PCAMotorController) Setup() {
	d.servos = make(map[string]*pca9685.Servo)
	for k, v := range d.Mapping {
		d.servos[k] = d.pca.ServoNew(v, d.ServoOptions)
	}
}

func (d *PCAMotorController) CalibrateAllJoints() {
	// No calibration is needed as servos always are at the correct position
}

func NewPCAMotorController() *PCAMotorController {
	i2c, err := i2c.New(pca9685.Address, "/dev/i2c-1")
	if err != nil {
		panic("Could not connect to i2c")
	}
	pca0, err := pca9685.New(i2c, nil)
	if err != nil {
		panic("Could not connect to pca9685")
	}
	return &PCAMotorController{
		pca: pca0,
		ServoOptions: &pca9685.ServOptions{
			AcRange:  pca9685.ServoRangeDef,
			MinPulse: pca9685.ServoMinPulseDef,
			MaxPulse: pca9685.ServoMaxPulseDef,
		},
	}
}
