package spotpuppy

import (
	"encoding/json"
	"math"
)

// LegIK describes types that can calculate motor rotations from a foot endpoint
type LegIK interface {
	// CalculateMotorRotations gets a list of motor rotations from a foot position relative to this leg.
	// The returned servo positions should be in the same order as GetMotorNames
	// (ie the first value of the rotations should be for the first servo name returned by GetMotorNames)
	CalculateMotorRotations(vector Vec3) []float64
	// GetMotorNames returns a list of the motor names of this leg. See CalculateMotorRotations for more info
	GetMotorNames() []string
	// GetRestingPosition returns the position of the foot of this leg such that the motor rotations are all 0
	GetRestingPosition() Vec3
	// LoadJson loads the json data into this object
	LoadJson(data []byte) error
}

// DirectMotorIK is an IK driver for 3 jointed legs (like Spot Mini), with one motor at each joint (no control rods or gears),
// and where the distance form foot to knee is the same as knee to hip
type DirectMotorIK struct {
	ReverseKneeJoint bool    `json:"reverse_knee_joint"`
	ReverseHipXJoint bool    `json:"reverse_hip_x_joint"`
	ReverseHipZJoint bool    `json:"reverse_hip_z_joint"`
	FlipXAxis        bool    `json:"flip_x_axis"`
	KneeOffset       float64 `json:"knee_offset"`
	HipXOffset       float64 `json:"hip_x_offset"`
	HipZOffset       float64 `json:"hip_z_offset"`
	// BoneLength is the length from the foot to the knee
	BoneLength float64 `json:"bone_length"`
}

// CalculateMotorRotations calculates the rotations of the three motors, and returns them in the order (hip left right, hip forwards backwords, knee)
func (dm *DirectMotorIK) CalculateMotorRotations(pos Vec3) []float64 {
	dist := pos.Len()
	if dist >= 2*dm.BoneLength {
		pos.Mul(1.999 * dm.BoneLength / dist)
	}
	kd := dm.kneeDegrees(dist) - 90
	hxd := (kd / 2) + dm.degreesBetween(pos.X, pos.Y) - 135
	hzd := dm.degreesBetween(pos.Z, pos.Y) - 90
	if dm.ReverseKneeJoint {
		kd = -kd
	}
	if dm.ReverseHipXJoint {
		hxd = -hxd
	}
	if dm.ReverseHipZJoint {
		hzd = -hzd
	}
	kd += dm.KneeOffset
	hxd += dm.HipXOffset
	hzd += dm.HipZOffset
	return []float64{clamp(hzd, -90, 90), clamp(hxd, -90, 90), clamp(kd, -90, 90)}
}
func clamp(x, mi, ma float64) float64 {
	if x < mi {
		return mi
	}
	if x > ma {
		return ma
	}
	return x
}
func (dm *DirectMotorIK) kneeDegrees(dist float64) float64 {
	ratio := dist / (2 * dm.BoneLength)
	if ratio > 1 {
		ratio = 1
	} else if ratio < 0 {
		ratio = 0
	}
	return (180.0 / 3.14159) * (2 * math.Asin(ratio))
}

func (dm *DirectMotorIK) degreesBetween(x, y float64) float64 {
	return math.Atan2(y, x) * (180.0 / 3.14159)
}

var directMotorIKNames = []string{"hip_z", "hip_x", "knee"}

func (dm *DirectMotorIK) GetMotorNames() []string {
	return directMotorIKNames
}

// GetRestingPosition returns the position where all of the joints are at 0 degrees
func (dm *DirectMotorIK) GetRestingPosition() Vec3 {
	return NewVector3(0, math.Sqrt(math.Pow(dm.BoneLength, 2)*2), 0)
}

// Loads json data into this object
func (dm *DirectMotorIK) LoadJson(data []byte) error {
	return json.Unmarshal(data, dm)
}

// NewDirectMotorIKGenerator creates a function to create empty DirectMotorIK structs. This is used by robot to create a seperate controller for each leg
func NewDirectMotorIKGenerator() func() LegIK {
	return func() LegIK {
		return &DirectMotorIK{
			ReverseKneeJoint: false,
			ReverseHipXJoint: false,
			ReverseHipZJoint: false,
			FlipXAxis:        false,
			KneeOffset:       0,
			HipXOffset:       0,
			HipZOffset:       0,
			BoneLength:       6,
		}
	}
}
