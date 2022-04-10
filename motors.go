package spotpuppy

// MotorController is an interface type to allow setting of positions to named motors
type MotorController interface {
	// SetMotor allows setting of a named motor to an angle between -90 to 90
	SetMotor(string, float64)
	// CreateMotorMapping is for initialising any motor mapping that is stored with all possible motor names
	CreateMotorMapping([]string)
	// Setup is called once the motor controller has a motor mapping set up and has been loaded from a file.
	// This could be used, for example, to create a servo object for each item in the mapping
	Setup()
}

// DummyMotorController is a basic MotorController. It does include a dummy servo mapping
type DummyMotorController struct {
	Mapping map[string]int
}

// NewDummyMotorController creates a new dummy motor controller
func NewDummyMotorController() *DummyMotorController {
	return &DummyMotorController{}
}

// SetMotor does nothing for dummy motor controller
func (d *DummyMotorController) SetMotor(s string, f float64) {
	//fmt.Println("Set motor " + s + "(" + strconv.Itoa(d.Mapping[s]) + ") to " + fmt.Sprintf("%f", f))
}

// CreateMotorMapping creates a map from string to int for this dummy. It then sets all servos to channel -1
func (d *DummyMotorController) CreateMotorMapping(names []string) {
	d.Mapping = make(map[string]int)
	for _, n := range names {
		d.Mapping[n] = -1
	}
}

// Setup does nothing for DummyMotorController
func (d *DummyMotorController) Setup() {

}
