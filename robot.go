package spotpuppy

import (
	"encoding/json"
	"os"

	m3 "github.com/JoshPattman/math3d"
)

const LegFrontLeft = "front_left"
const LegFrontRight = "front_right"
const LegBackLeft = "back_left"
const LegBackRight = "back_right"

type legs map[string]LegIK

// AllLegs is an ordered list of all the legs of a robot, useful for looping
var AllLegs = []string{LegFrontLeft, LegFrontRight, LegBackLeft, LegBackRight}

// Quadruped is a type to collect four LegIK objects, a motor controller, and some robot info together, to allow easy control
type Quadruped struct {
	Legs               legs            `json:"legs"`
	MotorController    MotorController `json:"motor_controller"`
	BodyDimensionX     float64         `json:"body_dimension_x"`
	BodyDimensionZ     float64         `json:"body_dimension_z"`
	cachedLegPositions map[string]m3.Vec3
	cachedLegRotations map[string][]float64
}

// ShoulderVec gets the m3.Vec3 between the robots center and the shoulder joint of the leg specified
func (q *Quadruped) ShoulderVec(leg string) m3.Vec3 {
	switch leg {
	case LegFrontLeft:
		return m3.V(q.BodyDimensionX/2, 0, q.BodyDimensionZ/2)
	case LegFrontRight:
		return m3.V(q.BodyDimensionX/2, 0, -q.BodyDimensionZ/2)
	case LegBackLeft:
		return m3.V(-q.BodyDimensionX/2, 0, q.BodyDimensionZ/2)
	case LegBackRight:
		return m3.V(-q.BodyDimensionX/2, 0, -q.BodyDimensionZ/2)
	default:
		return m3.V(0, 0, 0)
	}
}

// NewQuadruped creates a new quadruped from a function to create empty LegIK objects, and a MotorController.
// No objects in the robot will be set up correctly, and it is recommended that a file load is performed before anything else
func NewQuadruped(newIK func() LegIK, motorController MotorController) *Quadruped {
	return NewQuadrupedWithExtraMotors(newIK, motorController, []string{})
}

// NewQuadrupedWithExtraMotors creates a new quadruped from a function to create empty LegIK objects, and a MotorController. It also adds the specified servo names to the servo map.
// No objects in the robot will be set up correctly, and it is recommended that a file load is performed before anything else
func NewQuadrupedWithExtraMotors(newIK func() LegIK, motorController MotorController, extraMotors []string) *Quadruped {
	iks := make(map[string]LegIK)
	cachedLegPositions := make(map[string]m3.Vec3, 4)
	for _, l := range AllLegs {
		iks[l] = newIK()
		cachedLegPositions[l] = iks[l].GetRestingPosition()
	}
	names := make([]string, 0)
	for _, l := range AllLegs {
		for _, j := range iks[l].GetMotorNames() {
			names = append(names, l+"."+j)
		}
	}
	motorController.CreateMotorMapping(append(names, extraMotors...))
	return &Quadruped{
		Legs:               iks,
		MotorController:    motorController,
		cachedLegPositions: cachedLegPositions,
		cachedLegRotations: make(map[string][]float64),
	}
}

// SetExtraMotorNow sets the named motor to a position, without waiting for the next update loop
func (q *Quadruped) SetExtraMotorNow(motorName string, angle float64) {
	q.MotorController.SetMotor(motorName, angle)
}

// SaveToFile saves this quadruped to a file
func (q *Quadruped) SaveToFile(filename string) error {
	s, err := json.MarshalIndent(q, "", "\t")
	if err != nil {
		return err
	}
	err = os.WriteFile(filename, s, 0644)
	if err != nil {
		return err
	}
	return nil
}

// LoadFromFile loads some quadruped data from a file. It is important to make sure the quadruped that created that file had the same LegIK and MotorController types
func (q *Quadruped) LoadFromFile(filename string) error {
	s, err := os.ReadFile(filename)
	if err != nil {
		return err
	}
	err = json.Unmarshal(s, q)
	if err != nil {
		return err
	}
	q.MotorController.Setup()
	return nil
}

// UnmarshalJSON allows unmarshalling into the interface type LegIK
func (l *legs) UnmarshalJSON(data []byte) error {
	legs := make(map[string]json.RawMessage)
	err := json.Unmarshal(data, &legs)
	if err != nil {
		return err
	}
	for k, v := range legs {
		err = (*l)[k].LoadJson(v)
		if err != nil {
			return err
		}
	}
	return nil
}

// SetLegPosition sets a legs position for the next update call. It does not perform any calculations or movement immediately.
func (q *Quadruped) SetLegPosition(leg string, pos m3.Vec3) {
	q.cachedLegPositions[leg] = pos
}

// Update takes the most recent leg positions (set with SetLegPosition), calculates the motor angles with LegIK, and sets the motors with the MotorController
func (q *Quadruped) Update() {
	for _, l := range AllLegs {
		q.cachedLegRotations[l] = q.Legs[l].CalculateMotorRotations(q.cachedLegPositions[l])
	}
	for _, l := range AllLegs {
		r := q.cachedLegRotations[l]
		names := q.Legs[l].GetMotorNames()
		for i := range r {
			q.MotorController.SetMotor(l+"."+names[i], r[i])
		}
	}
}
