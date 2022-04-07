package spotpuppy

import "math"

type Vector3 struct {
	X float64
	Y float64
	Z float64
}

func NewVector3(x, y, z float64) *Vector3 {
	return &Vector3{
		X: x,
		Y: y,
		Z: z,
	}
}
func CopyVector3(v *Vector3) *Vector3 {
	return &Vector3{
		X: v.X,
		Y: v.Y,
		Z: v.Z,
	}
}

func (v *Vector3) Normalise() *Vector3 {
	return v.Mul(1.0 / v.Dist())
}

func (v *Vector3) Dist() float64 {
	return math.Sqrt((v.X * v.X) + (v.Y * v.Y) + (v.Z * v.Z))
}

func (v *Vector3) Mul(f float64) *Vector3 {
	vn := CopyVector3(v)
	vn.X *= f
	vn.Y *= f
	vn.Z *= f
	return vn
}

func (v *Vector3) Add(v2 *Vector3) *Vector3 {
	vn := CopyVector3(v)
	vn.X += v2.X
	vn.Y += v2.Y
	vn.Z += v2.Z
	return vn
}

func (v *Vector3) Inv() *Vector3 {
	vn := CopyVector3(v)
	vn.X *= -1
	vn.Y *= -1
	vn.Z *= -1
	return vn
}

var DirDown = Vector3{0, 1, 0}
var DirUp = DirDown.Inv()
var DirForward = Vector3{1, 0, 0}
var DirBackward = DirForward.Inv()
var DirLeft = Vector3{0, 0, 1}
var DirRight = DirLeft.Inv()
