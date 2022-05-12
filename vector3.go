package spotpuppy

import "math"

var DirDown = Vec3{0, 1, 0}
var DirUp = DirDown.Inv()
var DirForward = Vec3{1, 0, 0}
var DirBackward = DirForward.Inv()
var DirLeft = Vec3{0, 0, 1}
var DirRight = DirLeft.Inv()

type Vec3 struct {
	X float64
	Y float64
	Z float64
}

func NewVector3(x, y, z float64) Vec3 {
	return Vec3{
		X: x,
		Y: y,
		Z: z,
	}
}
func CopyVector3(v Vec3) Vec3 {
	return Vec3{
		X: v.X,
		Y: v.Y,
		Z: v.Z,
	}
}

func (v Vec3) Normalise() Vec3 {
	return v.Mul(1.0 / v.Dist())
}

func (v Vec3) Dist() float64 {
	return math.Sqrt((v.X * v.X) + (v.Y * v.Y) + (v.Z * v.Z))
}

func (v Vec3) Mul(f float64) Vec3 {
	vn := CopyVector3(v)
	vn.X *= f
	vn.Y *= f
	vn.Z *= f
	return vn
}

func (v Vec3) MulVec(v2 Vec3) Vec3 {
	vn := CopyVector3(v)
	vn.X *= v2.X
	vn.Y *= v2.Y
	vn.Z *= v2.Z
	return vn
}

func (v Vec3) Add(v2 Vec3) Vec3 {
	vn := CopyVector3(v)
	vn.X += v2.X
	vn.Y += v2.Y
	vn.Z += v2.Z
	return vn
}
func (v Vec3) Sub(v2 Vec3) Vec3 {
	vn := CopyVector3(v)
	vn.X -= v2.X
	vn.Y -= v2.Y
	vn.Z -= v2.Z
	return vn
}

func (v Vec3) Inv() Vec3 {
	vn := CopyVector3(v)
	vn.X *= -1
	vn.Y *= -1
	vn.Z *= -1
	return vn
}

func (v Vec3) Dot(v2 Vec3) float64 {
	return (v.X * v2.X) + (v.Y * v2.Y) + (v.Z * v2.Z)
}
func (v Vec3) Cross(v2 Vec3) *Vec3 {
	return &Vec3{
		v.Y*v2.Z - v.Z*v2.Y,
		v.Z*v2.X - v.X*v2.Z,
		v.X*v2.Y - v.Y*v2.X,
	}
}

// In converts this vector to be in a coordinate system. It is sugar for RollPitchCoordinateSystem.TransformDirection(Vec3)
func (v Vec3) In(quat Quat) Vec3 {
	return quat.Rotate(v)
}

func (v Vec3) AngleTo(v2 Vec3) float64 {
	return math.Acos(v.Dot(v2)/(v.Dist()*v2.Dist())) / math.Pi * 180.0
	//α = arccos[(a · b) / (|a| * |b|)]
}

func (v Vec3) ProjectToPlane(normal Vec3) Vec3 {
	d := v.Dot(normal) / normal.Dist()
	p := normal.Normalise().Mul(d)
	return v.Sub(p)
}
