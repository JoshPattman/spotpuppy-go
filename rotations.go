package spotpuppy

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
	rads := a * (math.Pi / 180.0)
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

func (q Quat) NoYaw() Quat {
	transformedFwd := q.Apply(DirForward)
	projected := transformedFwd.ProjectToPlane(DirUp)
	if projected.Len() > 0 {
		theta := math.Acos(projected.Unit().Dot(DirForward)) * 180.0 / 3.14159
		fmt.Println(theta)
		yawQuat := NewQuatAngleAxis(DirUp, theta)
		return q.RotateByGlobal(yawQuat.Inv()).Unit()
	}
	// Otherwise, the quaternion is pointing straight up, so just return it
	return q
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
