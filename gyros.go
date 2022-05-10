package spotpuppy

import "time"

// RotationSensor is an interface for getting the roll and pitch from a gyroscope/accelerometer
type RotationSensor interface {
	GetRollPitch() (float64, float64)
	Calibrate()
}

// DummyRotationSensor is a rotation sensor that does nothing
type DummyRotationSensor struct{}

// GetRollPitch returns 0, 0 for DummyRotationSensor
func (d *DummyRotationSensor) GetRollPitch() (float64, float64) {
	return 0, 0
}

// Calibrate does nothing for DummyRotationSensor
func (d *DummyRotationSensor) Calibrate() {

}

// NewDummyRotationSensor creates a new DummyRotationSensor
func NewDummyRotationSensor() *DummyRotationSensor {
	return &DummyRotationSensor{}
}

// ConcurrentRotationSensor uses a blocking rotation sensor but runs update at a steady rate in a goroutine, allowing for instant rotation accsess
type ConcurrentRotationSensor struct {
	R               RotationSensor
	UPS             float64
	rCache          float64
	pCache          float64
	needToCalibrate bool
}

// NewConcurrentRotationSensor creates a new ccrs from a rotaion sensor and a number of times to update per second
func NewConcurrentRotationSensor(r RotationSensor, ups float64) *ConcurrentRotationSensor {
	ccrs := &ConcurrentRotationSensor{
		R:               r,
		UPS:             ups,
		rCache:          0,
		pCache:          0,
		needToCalibrate: false,
	}
	go ccrsUpdateLoop(ccrs)
	return ccrs
}

// GetRollPitch returns the most recently updated roll and pitch for this rotation sensor
func (d *ConcurrentRotationSensor) GetRollPitch() (float64, float64) {
	return d.rCache, d.pCache
}

// Calibrate calibrates the underlying rotation sensor (after waiting for any updates it is running)
func (d *ConcurrentRotationSensor) Calibrate() {
	// Wait until other calibrations have completed
	for d.needToCalibrate {
	}
	// Set the calibration flag
	d.needToCalibrate = true
	time.Sleep(time.Millisecond * 10)
	// Wait for calibration to complete
	for d.needToCalibrate {
	}
}

func ccrsUpdateLoop(ccrs *ConcurrentRotationSensor) {
	lt := time.Now()
	for true {
		if ccrs.needToCalibrate {
			ccrs.R.Calibrate()
			ccrs.needToCalibrate = false
		}
		r, p := ccrs.R.GetRollPitch()
		ccrs.rCache = r
		ccrs.pCache = p
		for time.Since(lt).Seconds() < 1.0/ccrs.UPS {
		}
		lt = time.Now()
	}
}
