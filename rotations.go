package spotpuppy

import "github.com/westphae/quaternion"

func (o *RollPitchCoordinateSystem) TransformDirection(v *Vector3) *Vector3 {
	v3 := quaternion.Vec3{X: v.X, Y: v.Y, Z: v.Z}
	v3r := o.Rotation.RotateVec3(v3)
	return NewVector3(v3r.X, v3r.Y, v3r.Z)
}

func (o *RollPitchCoordinateSystem) SetRollPitch(roll, pitch float64) {
	dtor := 3.14159 / 180.0
	o.Rotation = quaternion.FromEuler(dtor*roll, 0, dtor*pitch)
}

func (o *RollPitchCoordinateSystem) UpdateRollPitchFrom(rs RotationSensor) {
	r, p := rs.GetRollPitch()
	o.SetRollPitch(r, p)
}

type RollPitchCoordinateSystem struct {
	Rotation quaternion.Quaternion
}

func NewRollPitchCoordinateSystem() *RollPitchCoordinateSystem {
	rpc := &RollPitchCoordinateSystem{}
	rpc.SetRollPitch(0, 0)
	return rpc
}

type RotationSensor interface {
	GetRollPitch() (float64, float64)
	Calibrate()
}
type DummyRotationSensor struct{}

func (d *DummyRotationSensor) GetRollPitch() (float64, float64) {
	return 0, 0
}
func (d *DummyRotationSensor) Calibrate() {

}
func NewDummyRotationSensor() *DummyRotationSensor {
	return &DummyRotationSensor{}
}
