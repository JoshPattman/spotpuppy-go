package spotpuppy

// Based on github.com/westphae/quaternion

import (
	"fmt"
	"math"
)

var QuatIdentity = NewQuat(1, 0, 0, 0)

// NewQuat returns a new quaternion
func NewQuat(w, x, y, z float64) Quat {
	return Quat{W: w, X: x, Y: y, Z: z}
}

// NewQuatAngleAxis returns a quaternion with rotation a in degrees around axis v
func NewQuatAngleAxis(v Vec3, a float64) Quat {
	v = v.Unit()
	rads := Radians(a)
	// Here we calculate the sin( theta / 2) once for optimization
	factor := math.Sin(rads / 2.0)

	// Calculate the x, y and z of the quaternion
	x := v.X * factor
	y := v.Y * factor
	z := v.Z * factor

	// Calculate the w value by cos( theta / 2 )
	w := math.Cos(rads / 2.0)

	return NewQuat(w, x, y, z).Unit()
}

func NewQuatFromTo(v1, v2 Vec3) Quat {
	axis := v1.Cross(v2)
	angle := v1.AngleTo(v2)
	return NewQuatAngleAxis(axis, angle)
}

// Quat represents a quaternion W+X*i+Y*j+Z*k
type Quat struct {
	W float64 // Scalar component
	X float64 // i component
	Y float64 // j component
	Z float64 // k component
}

// Conj returns the conjugate of a Quat (W,X,Y,Z) -> (W,-X,-Y,-Z)
func (qin Quat) Conj() Quat {
	qin.X = -qin.X
	qin.Y = -qin.Y
	qin.Z = -qin.Z
	return qin
}

// Norm2 returns the L2-Norm of a Quat (W,X,Y,Z) -> W*W+X*X+Y*Y+Z*Z
func (qin Quat) Norm2() float64 {
	return qin.W*qin.W + qin.X*qin.X + qin.Y*qin.Y + qin.Z*qin.Z
}

// Neg returns the negative
func (qin Quat) Neg() Quat {
	qin.W = -qin.W
	qin.X = -qin.X
	qin.Y = -qin.Y
	qin.Z = -qin.Z
	return qin
}

// Norm returns the L1-Norm of a Quat (W,X,Y,Z) -> Sqrt(W*W+X*X+Y*Y+Z*Z)
func (qin Quat) Norm() float64 {
	return math.Sqrt(qin.Norm2())
}

func quatProd(qin ...Quat) Quat {
	qout := Quat{1, 0, 0, 0}
	var w, x, y, z float64
	for _, q := range qin {
		w = qout.W*q.W - qout.X*q.X - qout.Y*q.Y - qout.Z*q.Z
		x = qout.W*q.X + qout.X*q.W + qout.Y*q.Z - qout.Z*q.Y
		y = qout.W*q.Y + qout.Y*q.W + qout.Z*q.X - qout.X*q.Z
		z = qout.W*q.Z + qout.Z*q.W + qout.X*q.Y - qout.Y*q.X
		qout = Quat{w, x, y, z}
	}
	return qout
}

// Prod returns the product of q and q2 (q*q2)
func (q Quat) Prod(q2 Quat) Quat {
	return Quat{
		q.W*q2.W - q.X*q2.X - q.Y*q2.Y - q.Z*q2.Z,
		q.W*q2.X + q.X*q2.W + q.Y*q2.Z - q.Z*q2.Y,
		q.W*q2.Y + q.Y*q2.W + q.Z*q2.X - q.X*q2.Z,
		q.W*q2.Z + q.Z*q2.W + q.X*q2.Y - q.Y*q2.X,
	}
}

// Unit returns the Quat rescaled to unit-L1-norm
func (qin Quat) Unit() Quat {
	k := qin.Norm()
	return Quat{qin.W / k, qin.X / k, qin.Y / k, qin.Z / k}
}

// Inv returns the Quat conjugate rescaled so that Q Q* = 1
func (qin Quat) Inv() Quat {
	k2 := qin.Norm2()
	q := qin.Conj()
	return Quat{q.W / k2, q.X / k2, q.Y / k2, q.Z / k2}
}

// Apply returns the vector rotated by the quaternion.
func (qin Quat) Apply(vec Vec3) Vec3 {
	conj := qin.Conj()
	aug := Quat{0, vec.X, vec.Y, vec.Z}
	rot := quatProd(qin, aug, conj)
	return Vec3{rot.X, rot.Y, rot.Z}
}

// RotateByGlobal rotates quaternion a by b along the local axis relative to a
func (a Quat) RotateByLocal(b Quat) Quat {
	return a.Prod(b)
}

// RotateByGlobal rotates quaternion a by b along the global axis
func (a Quat) RotateByGlobal(b Quat) Quat {
	return b.Prod(a)
}

// NoYaw removes the global yaw component of this quaternion.
// Use this to take a quaternion in global space and make it useful for robot calculations. This suffers from gimbal lock
func (q Quat) NoYaw() Quat {
	heading := q.HeadingAngle()
	correction := NewQuatAngleAxis(Up, heading).Inv()
	return q.RotateByGlobal(correction)
}

// Returns the heading angle of this quaternion in degrees. This suffers from gimbal lock
func (q Quat) HeadingAngle() float64 {
	fwdDir := Forward.Rotated(q).ProjectToPlane(Up)
	// Ensure that the forward direction is not pointing straight up
	if fwdDir.Len() == 0 {
		return 0
	}
	fwdDir = fwdDir.Unit()
	return Degrees(math.Atan2(fwdDir.Z, fwdDir.X))
}

// String converts this quaternion to a string
func (q Quat) String() string {
	return fmt.Sprintf("(w%.2f,x%.2f,y%.2f,z%.2f)", q.W, q.X, q.Y, q.Z)
}

// AxesRemapper allows remapping of vectors from one axis space to another
type AxesRemapper struct {
	SrcForwardVector Vec3 `json:"src-forward"`
	SrcUpVector      Vec3 `json:"src-up"`
	SrcLeftVector    Vec3 `json:"src-left"`

	TargetForwardVector Vec3 `json:"target-forward"`
	TargetUpVector      Vec3 `json:"target-up"`
	TargetLeftVector    Vec3 `json:"target-left"`
}

// Returns an AxesRemapper with source vectors forward, up, and left. These are the vectors that represent the axes in the system we will convert from. By default, the target axes are spotpuppy Forward, Up, Left
func NewAxesRemapper(SrcForwardVector, SrcUpVector, SrcLeftVector Vec3) *AxesRemapper {
	return &AxesRemapper{
		SrcForwardVector, SrcUpVector, SrcLeftVector,
		Forward, Up, Left,
	}
}

// Remap the vector from coordinate system with Src axes to coordinate system with Target axes, whilst keeping the meaning of the vector
func (a *AxesRemapper) Remap(v Vec3) Vec3 {
	fwd := v.Dot(a.SrcForwardVector)
	lft := v.Dot(a.SrcLeftVector)
	up := v.Dot(a.SrcUpVector)
	return a.TargetForwardVector.Mul(fwd).Add(a.TargetLeftVector.Mul(lft)).Add(a.TargetUpVector.Mul(up))
}

// Remap the vector from coordinate system with Target axes to coordinate system with Src axes, whilst keeping the meaning of the vector
func (a *AxesRemapper) RemapInverse(v Vec3) Vec3 {
	fwd := v.Dot(a.TargetForwardVector)
	lft := v.Dot(a.TargetLeftVector)
	up := v.Dot(a.TargetUpVector)
	return a.SrcForwardVector.Mul(fwd).Add(a.SrcLeftVector.Mul(lft)).Add(a.SrcUpVector.Mul(up))
}

/*
// Euler returns the Euler angles phi, theta, psi corresponding to a Quaternion
func (q Quat) Euler() (float64, float64, float64) {
	r := q.Unit()
	phi := math.Atan2(2*(r.W*r.X+r.Y*r.Z), 1-2*(r.X*r.X+r.Y*r.Y))
	theta := math.Asin(2 * (r.W*r.Y - r.Z*r.X))
	psi := math.Atan2(2*(r.X*r.Y+r.W*r.Z), 1-2*(r.Y*r.Y+r.Z*r.Z))
	return phi, theta, psi
}

// FromEuler returns a Quaternion corresponding to Euler angles phi, theta, psi
func FromEuler(phi, theta, psi float64) Quat {
	q := Quat{}
	q.W = math.Cos(phi/2)*math.Cos(theta/2)*math.Cos(psi/2) +
		math.Sin(phi/2)*math.Sin(theta/2)*math.Sin(psi/2)
	q.X = math.Sin(phi/2)*math.Cos(theta/2)*math.Cos(psi/2) -
		math.Cos(phi/2)*math.Sin(theta/2)*math.Sin(psi/2)
	q.Y = math.Cos(phi/2)*math.Sin(theta/2)*math.Cos(psi/2) +
		math.Sin(phi/2)*math.Cos(theta/2)*math.Sin(psi/2)
	q.Z = math.Cos(phi/2)*math.Cos(theta/2)*math.Sin(psi/2) -
		math.Sin(phi/2)*math.Sin(theta/2)*math.Cos(psi/2)
	return q
}
*/
