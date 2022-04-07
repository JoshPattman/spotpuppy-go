package spotpuppy

import (
	"encoding/json"
	"os"
)

const LegFrontLeft = "front_left"
const LegFrontRight = "front_right"
const LegBackLeft = "back_left"
const LegBackRight = "back_right"

// Legs is an extension of a map from string to LegIK that allows for json unmarshalling
type Legs map[string]LegIK

// AllLegs is an ordered list of all the legs of a robot, useful for looping
var AllLegs = []string{LegFrontLeft, LegFrontRight, LegBackLeft, LegBackRight}

// Quadruped is a type to collect four LegIK objects, a motor controller, and some robot info together, to allow easy control
type Quadruped struct {
	Legs               Legs            `json:"legs"`
	MotorController    MotorController `json:"motor_controller"`
	BodyDimensionX     float64         `json:"body_dimension_x"`
	BodyDimensionZ     float64         `json:"body_dimension_z"`
	cachedLegPositions map[string]*Vector3
	cachedLegRotations map[string][]float64
}

// GetVectorToShoulder gets the Vector3 between the robots center and the shoulder joint of the leg specified
func (q *Quadruped) GetVectorToShoulder(leg string) *Vector3 {
	switch leg {
	case LegFrontLeft:
		return NewVector3(q.BodyDimensionX/2, 0, q.BodyDimensionZ/2)
	case LegFrontRight:
		return NewVector3(-q.BodyDimensionX/2, 0, q.BodyDimensionZ/2)
	case LegBackLeft:
		return NewVector3(q.BodyDimensionX/2, 0, -q.BodyDimensionZ/2)
	case LegBackRight:
		return NewVector3(-q.BodyDimensionX/2, 0, -q.BodyDimensionZ/2)
	default:
		return NewVector3(0, 0, 0)
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
	cachedLegPositions := make(map[string]*Vector3, 4)
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
	motorController.CreateServoMapping(append(names, extraMotors...))
	return &Quadruped{
		Legs:               iks,
		MotorController:    motorController,
		cachedLegPositions: cachedLegPositions,
		cachedLegRotations: make(map[string][]float64),
	}
}

func (q *Quadruped) SetExtraMotorNow(motorName string, angle float64) {
	q.MotorController.SetMotor(motorName, angle)
}

// SaveToFile saves this quadruped to a file
func (q *Quadruped) SaveToFile(filename string) {
	s, err := json.MarshalIndent(q, "", "\t")
	if err != nil {
		panic(err)
	}
	err = os.WriteFile(filename, s, 0644)
	if err != nil {
		panic(err)
	}
}

// LoadFromFile loads some quadruped data from a file. It is important to make sure the quadruped that created that file had the same LegIK and MotorController types
func (q *Quadruped) LoadFromFile(filename string) {
	s, err := os.ReadFile(filename)
	if err != nil {
		panic(err)
	}
	err = json.Unmarshal(s, q)
	if err != nil {
		panic(err)
	}
	q.MotorController.Setup()
}

// UnmarshalJSON allows unmarshalling into the interface type LegIK
func (l *Legs) UnmarshalJSON(data []byte) error {
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
func (q *Quadruped) SetLegPosition(leg string, pos *Vector3) {
	q.cachedLegPositions[leg] = pos
}

// Update takes the most recent leg positions (set with SetLegPosition), calculates the motor angles with LegIK, and sets the motors with the MotorController
func (q *Quadruped) Update() {
	for _, l := range AllLegs {
		q.cachedLegRotations[l] = q.Legs[l].SetEndpoint(q.cachedLegPositions[l])
	}
	for _, l := range AllLegs {
		r := q.cachedLegRotations[l]
		names := q.Legs[l].GetMotorNames()
		for i := range r {
			q.MotorController.SetMotor(l+"."+names[i], r[i])
		}
	}
}
