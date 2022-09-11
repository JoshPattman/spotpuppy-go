package spotpuppy

import (
	"fmt"
	"math"
)

// These are for compatibility. Dont use them
var (
	// Downwards unit vector. DEPRECATED: USE `Down`
	DirDown = Vec3{0, 1, 0}
	// Upwards unit vector. DEPRECATED: USE `Up`
	DirUp = DirDown.Inv()
	// Forwards unit vector. DEPRECATED: USE `Forward`
	DirForward = Vec3{1, 0, 0}
	// Backwards unit vector. DEPRECATED: USE `Backward`
	DirBackward = DirForward.Inv()
	// Left unit vector. DEPRECATED: USE `Left`
	DirLeft = Vec3{0, 0, 1}
	// Right unit vector. DEPRECATED: USE `Right`
	DirRight = DirLeft.Inv()
)

var (
	// Downwards unit vector
	Down = DirDown
	// Upwards unit vector
	Up = DirUp
	// Left unit vector
	Left = DirLeft
	// Right unit vector
	Right = DirRight
	// Forwards unit vector
	Forward = DirForward
	// Backwards unit vector
	Backward = DirBackward
	// Zero vector
	Zero = Vec3{0, 0, 0}
)

// A Vector with 3 dimensions X Y Z
type Vec3 struct {
	X float64
	Y float64
	Z float64
}

// Create a new Vec3 with x y z components
func NewVector3(x, y, z float64) Vec3 {
	return Vec3{
		X: x,
		Y: y,
		Z: z,
	}
}

// Create a new Vec3 with x y z components. Sugar for NewVector3
func V3(x, y, z float64) Vec3 {
	return NewVector3(x, y, z)
}

// Return the normalized form of this vector. If the length is 0, it will return Zero
func (v Vec3) Unit() Vec3 {
	l := v.Len()
	if l == 0 {
		return Zero
	}
	return v.Mul(1.0 / l)
}

// Returns the length of this vector
func (v Vec3) Len() float64 {
	return math.Sqrt((v.X * v.X) + (v.Y * v.Y) + (v.Z * v.Z))
}

// Returns the vector scaled by a scalar number
func (v Vec3) Mul(f float64) Vec3 {
	vn := v
	vn.X *= f
	vn.Y *= f
	vn.Z *= f
	return vn
}

// Returns the elementwise multiplication of this vector and the parameter vector
func (v Vec3) MulVec(v2 Vec3) Vec3 {
	vn := v
	vn.X *= v2.X
	vn.Y *= v2.Y
	vn.Z *= v2.Z
	return vn
}

// Returns the elementwise addition of this vector and the parameter vector
func (v Vec3) Add(v2 Vec3) Vec3 {
	vn := v
	vn.X += v2.X
	vn.Y += v2.Y
	vn.Z += v2.Z
	return vn
}

// Returns the elementwise subtraction of this vector and the parameter vector
func (v Vec3) Sub(v2 Vec3) Vec3 {
	vn := v
	vn.X -= v2.X
	vn.Y -= v2.Y
	vn.Z -= v2.Z
	return vn
}

// Returns the inverse of this vector: v.Mul(-1)
func (v Vec3) Inv() Vec3 {
	return v.Mul(-1)
}

// Returns the dot product of this vector and the parameter vector
func (v Vec3) Dot(v2 Vec3) float64 {
	return (v.X * v2.X) + (v.Y * v2.Y) + (v.Z * v2.Z)
}

// Returns the cross product of this vector and the parameter vector
func (v Vec3) Cross(v2 Vec3) Vec3 {
	return Vec3{
		v.Y*v2.Z - v.Z*v2.Y,
		v.Z*v2.X - v.X*v2.Z,
		v.X*v2.Y - v.Y*v2.X,
	}
}

// Returns the vector rotated by parameter quaternion
func (v Vec3) Rotated(q Quat) Vec3 {
	return q.Apply(v)
}

// Returns (in degrees) the angle from this vector and the parameter vector
func (v Vec3) AngleTo(v2 Vec3) float64 {
	return Degrees(math.Acos(v.Dot(v2) / (v.Len() * v2.Len())))
}

// Returns this vector projected onto the plane, where the normal of the plane is the parameter vector
func (v Vec3) ProjectToPlane(normal Vec3) Vec3 {
	d := v.Dot(normal) / normal.Len()
	p := normal.Unit().Mul(d)
	return v.Sub(p)
}

// Returns this vector as a readable string
func (v Vec3) String() string {
	return fmt.Sprintf("(x%.2f,y%.2f,z%.2f)", v.X, v.Y, v.Z)
}

// Converts radians to degrees
func Degrees(x float64) float64 {
	return x * 180.0 / math.Pi
}

// Converts degrees to radians
func Radians(x float64) float64 {
	return x * math.Pi / 180.0
}
