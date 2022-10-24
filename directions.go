package spotpuppy

import (
	m3 "github.com/JoshPattman/math3d"
)

// These are for compatibility. Dont use them
var (
	// Downwards unit vector. DEPRECATED: USE `Down`
	DirDown = m3.V(0, 1, 0)
	// Upwards unit vector. DEPRECATED: USE `Up`
	DirUp = DirDown.Inv()
	// Forwards unit vector. DEPRECATED: USE `Forward`
	DirForward = m3.V(1, 0, 0)
	// Backwards unit vector. DEPRECATED: USE `Backward`
	DirBackward = DirForward.Inv()
	// Left unit vector. DEPRECATED: USE `Left`
	DirLeft = m3.V(0, 0, 1)
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
	Zero = m3.V(0, 0, 0)
)

// AxesRemapper allows remapping of vectors from one axis space to another
type AxesRemapper struct {
	SrcForwardVector m3.Vec3 `json:"src-forward"`
	SrcUpVector      m3.Vec3 `json:"src-up"`
	SrcLeftVector    m3.Vec3 `json:"src-left"`

	TargetForwardVector m3.Vec3 `json:"target-forward"`
	TargetUpVector      m3.Vec3 `json:"target-up"`
	TargetLeftVector    m3.Vec3 `json:"target-left"`
}

// Returns an AxesRemapper with source vectors forward, up, and left. These are the vectors that represent the axes in the system we will convert from. By default, the target axes are spotpuppy Forward, Up, Left
func NewAxesRemapper(SrcForwardVector, SrcUpVector, SrcLeftVector m3.Vec3) *AxesRemapper {
	return &AxesRemapper{
		SrcForwardVector, SrcUpVector, SrcLeftVector,
		Forward, Up, Left,
	}
}

// Remap the vector from coordinate system with Src axes to coordinate system with Target axes, whilst keeping the meaning of the vector
func (a *AxesRemapper) Remap(v m3.Vec3) m3.Vec3 {
	fwd := v.Dot(a.SrcForwardVector)
	lft := v.Dot(a.SrcLeftVector)
	up := v.Dot(a.SrcUpVector)
	return a.TargetForwardVector.Mul(fwd).Add(a.TargetLeftVector.Mul(lft)).Add(a.TargetUpVector.Mul(up))
}

// Remap the vector from coordinate system with Target axes to coordinate system with Src axes, whilst keeping the meaning of the vector
func (a *AxesRemapper) RemapInverse(v m3.Vec3) m3.Vec3 {
	fwd := v.Dot(a.TargetForwardVector)
	lft := v.Dot(a.TargetLeftVector)
	up := v.Dot(a.TargetUpVector)
	return a.SrcForwardVector.Mul(fwd).Add(a.SrcLeftVector.Mul(lft)).Add(a.SrcUpVector.Mul(up))
}
