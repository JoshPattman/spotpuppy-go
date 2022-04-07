package spotpuppy

import (
	"encoding/json"
	"os"
)

const LegFrontLeft = "front_left"
const LegFrontRight = "front_right"
const LegBackLeft = "back_left"
const LegBackRight = "back_right"

type Legs map[string]LegIK

var AllLegs = []string{LegFrontLeft, LegFrontRight, LegBackLeft, LegBackRight}

type Quadruped struct {
	Legs               Legs            `json:"legs"`
	MotorController    MotorController `json:"motor_controller"`
	BodyDimensionX     float64         `json:"body_dimension_x"`
	BodyDimensionZ     float64         `json:"body_dimension_z"`
	cachedLegPositions map[string]*Vector3
	cachedLegRotations map[string][]float64
}

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

func NewQuadruped(newIK func() LegIK, motorController MotorController) *Quadruped {
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
	motorController.CreateServoMapping(names)
	return &Quadruped{
		Legs:               iks,
		MotorController:    motorController,
		cachedLegPositions: cachedLegPositions,
		cachedLegRotations: make(map[string][]float64),
	}
}

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

func (q *Quadruped) SetLegPosition(leg string, pos *Vector3) {
	q.cachedLegPositions[leg] = pos
}

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
