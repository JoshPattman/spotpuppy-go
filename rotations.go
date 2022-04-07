package spotpuppy

import (
	"github.com/westphae/quaternion"
	"time"
)

// TransformDirection takes a vector in world space and rotates it so it is in this coordinate system space
func (o *RollPitchCoordinateSystem) TransformDirection(v *Vector3) *Vector3 {
	v3 := quaternion.Vec3{X: v.X, Y: v.Y, Z: v.Z}
	v3r := o.Rotation.RotateVec3(v3)
	return NewVector3(v3r.X, v3r.Y, v3r.Z)
}

// TD is shorthand for TransformDirection
func (o *RollPitchCoordinateSystem) TD(v *Vector3) *Vector3 {
	return o.TransformDirection(v)
}

// SetRollPitch sets the roll and pitch of this coordinate system
func (o *RollPitchCoordinateSystem) SetRollPitch(roll, pitch float64) {
	dtor := 3.14159 / 180.0
	o.Rotation = quaternion.FromEuler(dtor*roll, 0, dtor*pitch)
}

// UpdateRollPitchFrom gets the roll and pitch from a rotation sensor and copies it into this
func (o *RollPitchCoordinateSystem) UpdateRollPitchFrom(rs RotationSensor) {
	r, p := rs.GetRollPitch()
	o.SetRollPitch(r, p)
}

// RollPitchCoordinateSystem is a way to easily use roll and pitch to rotate directions
type RollPitchCoordinateSystem struct {
	Rotation quaternion.Quaternion
}

// NewRollPitchCoordinateSystem creates a new system with 0 roll and pitch
func NewRollPitchCoordinateSystem() *RollPitchCoordinateSystem {
	rpc := &RollPitchCoordinateSystem{}
	rpc.SetRollPitch(0, 0)
	return rpc
}

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

// CachedConcurrentRotationSensor uses a blocking rotation sensor but runs update at a steady rate in a goroutine, allowing for instant rotation accsess
type CachedConcurrentRotationSensor struct {
	R               RotationSensor
	UPS             float64
	rCache          float64
	pCache          float64
	needToCalibrate bool
}

func NewCachedConcurrentRotationSensor(r RotationSensor, ups float64) *CachedConcurrentRotationSensor {
	ccrs := &CachedConcurrentRotationSensor{
		R:               r,
		UPS:             ups,
		rCache:          0,
		pCache:          0,
		needToCalibrate: false,
	}
	go ccrsUpdateLoop(ccrs)
	return ccrs
}
func (d *CachedConcurrentRotationSensor) GetRollPitch() (float64, float64) {
	return d.rCache, d.pCache
}
func (d *CachedConcurrentRotationSensor) Calibrate() {
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

func ccrsUpdateLoop(ccrs *CachedConcurrentRotationSensor) {
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
