package spotpuppy

import (
	"math"
)

// NewQuat returns a new quaternion
func NewQuat(w, x, y, z float64) Quat {
	return Quat{W: w, X: x, Y: y, Z: z}
}

// NewQuatPure returns a new pure quaternion (no scalar part)
func NewQuatPure(x, y, z float64) Quat {
	return Quat{X: x, Y: y, Z: z}
}

// NewQuatEuler returns a Quat corresponding to Euler angles phi, theta, psi
func NewQuatEuler(phi, theta, psi float64) Quat {
	phi, theta, psi = phi*math.Pi/180.0, theta*math.Pi/180.0, psi*math.Pi/180.0
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

func NewQuatRollPitch(roll, pitch float64) Quat {
	pitchRot := NewQuatAngleAxis(DirLeft, pitch)
	rollRot := NewQuatAngleAxis(DirForward, pitch)
	q := NewQuat(1, 0, 0, 0)
	// Rotate roll in world space
	q = rollRot.Prod(q)
	// Rotate pitch in world space
	q = pitchRot.Prod(q)
	return q
}

func NewQuatAngleAxis(v Vec3, a float64) Quat {
	// Here we calculate the sin( theta / 2) once for optimization
	factor := math.Sin(a * (math.Pi / 180.0) / 2.0)

	// Calculate the x, y and z of the quaternion
	x := v.X * factor
	y := v.Y * factor
	z := v.Z * factor

	// Calculate the w value by cos( theta / 2 )
	w := math.Cos(a * (math.Pi / 180.0) / 2.0)

	return NewQuat(x, y, z, w).Unit()
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

/*// Sum returns the vector sum of any number of Quaternions
func Sum(qin ...Quat) Quat {
	qout := Quat{}
	for _, q := range qin {
		qout.W += q.W
		qout.X += q.X
		qout.Y += q.Y
		qout.Z += q.Z
	}
	return qout
}*/

// Prod returns the non-commutative product of any number of Quaternions
func QuatProd(qin ...Quat) Quat {
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

// Rotate returns the vector rotated by the quaternion.
func (qin Quat) Rotate(vec *Vec3) *Vec3 {
	conj := qin.Conj()
	aug := Quat{0, vec.X, vec.Y, vec.Z}
	rot := QuatProd(qin, aug, conj)
	return &Vec3{rot.X, rot.Y, rot.Z}
}

// Euler returns the Euler angles phi, theta, psi corresponding to a Quat
func (q Quat) Euler() (float64, float64, float64) {
	r := q.Unit()
	phi := math.Atan2(2*(r.W*r.X+r.Y*r.Z), 1-2*(r.X*r.X+r.Y*r.Y)) * 180.0 / math.Pi
	theta := math.Asin(2*(r.W*r.Y-r.Z*r.X)) * 180.0 / math.Pi
	psi := math.Atan2(2*(r.X*r.Y+r.W*r.Z), 1-2*(r.Y*r.Y+r.Z*r.Z)) * 180.0 / math.Pi
	return phi, theta, psi
}

// RotMat returns the rotation matrix (as float array) corresponding to a Quat
func (qin Quat) RotMat() [3][3]float64 {
	q := qin.Unit()
	m := [3][3]float64{}
	m[0][0] = 1 - 2*(q.Y*q.Y+q.Z*q.Z)
	m[0][1] = 2 * (q.X*q.Y - q.W*q.Z)
	m[0][2] = 2 * (q.W*q.Y + q.X*q.Z)

	m[1][1] = 1 - 2*(q.Z*q.Z+q.X*q.X)
	m[1][2] = 2 * (q.Y*q.Z - q.W*q.X)
	m[1][0] = 2 * (q.W*q.Z + q.Y*q.X)

	m[2][2] = 1 - 2*(q.X*q.X+q.Y*q.Y)
	m[2][0] = 2 * (q.Z*q.X - q.W*q.Y)
	m[2][1] = 2 * (q.W*q.X + q.Z*q.Y)
	return m
}

func (q Quat) GetRollPitch() (float64, float64) {
	localFwd := q.Rotate(&DirForward)
	localLft := q.Rotate(&DirLeft)
	// Global up will always point up in global space
	globalUp := DirUp
	// Global left will always have zero y component but will be at right angle to global forward
	globalLft := globalUp.Cross(localFwd)
	// Global forward is the vector going forward in the same direction as local forward but with zero pitch
	globalFwd := globalUp.Cross(globalLft)

	return localLft.AngleTo(globalLft), localFwd.AngleTo(globalFwd)
}
