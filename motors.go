package spotpuppy

type MotorController interface {
	SetMotor(string, float64)
	CreateServoMapping([]string)
	Setup()
}

type DummyMotorController struct {
	Mapping map[string]int
}

func NewDummyMotorController() *DummyMotorController {
	return &DummyMotorController{}
}

func (d *DummyMotorController) SetMotor(s string, f float64) {
	//fmt.Println("Set motor " + s + "(" + strconv.Itoa(d.Mapping[s]) + ") to " + fmt.Sprintf("%f", f))
}

func (d *DummyMotorController) CreateServoMapping(names []string) {
	d.Mapping = make(map[string]int)
	for _, n := range names {
		d.Mapping[n] = -1
	}
}

func (d *DummyMotorController) Setup() {

}
