# `spotpuppy-go` - The quadruped control module for go
## What does this module do?
* Makes coding walking algorithms for quadrupedal robots more readable, yet still gives the programmer enough freedom to create complex programs
* Makes it easy to use algorithms between mechanically different robots - when creating a robot you have to provide an IK controller. This can easily be changed to allow lots of different leg configurations.
	* This concept also extends to motor controller drivers and rotation sensors. Both of these are interface types and can be switched out for either pre-build ones, or custom implementations.
* Allows you to work with vectors (such as `up` or `forward`)
	* This is much more intuitive than bare coordinates, and much more readable tha directly controlling motor rotations
* Takes care of saving and loading robot configuration to disk - including custom implementations of motor controllers and leg IK drivers
## What robots can run this code?
There are some limitations
* You have to be able to build golang to the platform (I run all of my robots of raspberry pis)
* The robot must have four legs, arranged such that there is one at front right, front left, back right, and back left (similar to boston dynamics spot mini, not a spider like quadruped)

Other than these limitations, you can write motor controllers, rotation sensors, and leg ik drivers for basically anything you can think of, and these components will plug into the module with ease.
## Examples
There is a repo with some example code using this module to make a robot _walk_ [here](github.com/joshpattman/spotpuppy-go-example).
## Usage
The below code (15 lines!) is all you would need to write to create a robot, then keep its feet at the same points on the floor no matter what rotation the body is at. This particular robot has a `3 servo, one servo in each joint` leg design, a `pca9685` motor driver, and an arduino running arduinodmp code (The sketch can be found on the `simple` branch [here](github.com/joshpattman/arduino-mpu6050)).
```go
// Create a quadruped with direct motor IKs and a pca9685 motor controller
q := sp.NewQuadruped(spotpuppy.NewDirectMotorIKGenerator(), pca9685.NewPCAMotorController())
// Load its parameters from config file
q.LoadFromFile("config.json")

// Create a rotation sensor over a serrial connection to an arduino
mpu := arduinompu.NewArduinoRotationSensor("/dev/ttyUSB0")

// Create a new coordinate system which will be used to represent the floor
// This is used in code to rotate vectors from robot space to global space
cs := spotpuppy.NewRollPitchCoordinateSystem()

// Create a ups timer. This will allow us to make the robot update at 100 times per second
// This is particularly useful for PID control loops, as they work best at fixed update rates
ups := spotpuppy.NewUPSTimer(100)

// This is our update loop and gets run at 100 times per second
for true {
	// Copy the current rotation of the robot into the coordinate system
	cs.UpdateRollPitchFrom(mpu)
	// Find the vector that is 6cm down in robot space, then rotate it to world space
	// This means that it will always point at the floor, no matter what rotation the body is at
	straightDown := cs.TransformDirection(spotpuppy.DirDown.Mul(6))
	// For each leg (l is a string which can be used to identify legs)
	for _, l := range spotpuppy.AllLegs {
		// Vector from this legs shoulder to the center of the body, in robot space
		shoulderRobotSpace := q.ShoulderVec(l)
		// Vector from the robots center to the shoulder, if the body was flat in world space
		shoulderInvWorldSpace := cs.TransformDirection(q.ShoulderVec(l).Inv())
		// Add them all together to get the foot position, then set that foot
		q.SetLegPosition(l, shoulderRobotSpace.Add(straightDown).Add(shoulderInvWorldSpace))
	}
	// Calculate all motor positions and push them to the motor controller
	q.Update()
	// Wait until its time to do the next update
	ups.WaitForNext()
}
```
## Included types
### LegIk
* `DirectMotorIK` - This is an IK driver for a leg with three motors, one at each joint, and joints laid out in the same location as Boston Dynamics Spot Mini (`knee`, `hip_x` (hip forwards and backwards), `hip_z` (hip left and right))
### MotorController
* `DummyMotorController` - This does nothing. It is there as a placeholder for performance testing
* `PCAMotorController` - This is a motor controller designed to interface with the pca9685 servo controller. Tested only on rpi4
### RotationSensor
* `DummyRotationSensor` - This does nothing. It is there as a placeholder for performance testing
* `ArduinoRotationSensor` - This connects to an arduino over serial that is running `simple` branch of [this](github.com/joshpattman/arduino-mpu6050) repo. The arduino is then connected to an mpu6050
## Custom type implementations
### LegIK
A `LegIK` controller describes a type that takes am input `(x,y,z)` in space relative to the leg, and returns a number of motor rotations. Some example coordinates:
* `(0,0,0)` is the position at the shoulder
* `(0,1,0)` is 1cm down from the shoulder
* `(1,0,0)` is 1cm forward from the shoulder
* `(0,0,1)` is 1cm left from the shoulder
```go
type LegIK interface {
	// This function takes in some coordinates for the foot, and returns a list of motor rotations
	// There can be as many motors as you want, as long as they are all declared in GetMotorNames()
	CalculateMotorRotations(vector *Vector3) []float64
	// This just returns a list of all of the named motors in the leg
	// For example, the direct mortor ik returns ["hip_x","hip_z","knee"]
	// The order of these is the same order that the rotations are returned in CalculateMotorRotations()
	GetMotorNames() []string
	// This returns the position the foor should be at to have all motors centered
	GetRestingPosition() *Vector3
	// Load the json data into this object
	LoadJson(data []byte) error
}
```
### MotorController
A motor controller describes a type that can set the positions of named motors to rotations (from -90 to 90)
> Note on motor mappings: A motor mapping is usually a `map[string]int`, where the key represents a named motor, and the value represents an output channel (eg `servo on pin 8`)
```go
type MotorController interface {
	// Set a named motor to an angle between -90 to 90
	SetMotor(string, float64)
	// Create the motor mapping using  a list of all possible motor names
	// This should set all of the motors to an invalid channel (eg -1)
	CreateMotorMapping([]string)
	// This function is called after both CreateServoMapping and loading the motor mapping from disk
	// This can be used, for example, to create Servo objects for each mmotor in the mapping
	Setup()
}
```
### RotationSensor
A rotation sensor describes a type which can get the roll and pitch of the robot
```go
type RotationSensor interface {
	// This returns the roll and pitch, in degrees, of the robot
	// 0,0 means the robot is flat. This should be blocking.
	// If concurrency is desired, you can wrap this type in ConcurrentRotationSensor
	GetRollPitch() (float64, float64)
	// This is called to calibrate the rotation sensor
	// It should block until calibration is complete
	Calibrate()
}
```
## Differences to spotpuppy python
You may have noticed that this module shares its name with the `spotpuppy` python library (`github.com/joshpattman/spotpuppy`). This module is inspired by that package, but some less efficient or intuitive areas have been redesigned for maximum simplicity and performance. Some of the changes are:
- Lack of a robot type to extend. You will have to write this type from scratch, however i have found that this actually shortens code and increases readability
- LegIK interface. You can now write custom IK controllers for the legs which allow much easier integration with other leg designs
- Simpler saving/loading. There is now one config file containing everything, with much more concise json
- Faster performance. From some (not very in depth tests), i think this package runs at least 50-100 times faster than the other code (this is not necessarily all pythons fault, and is partly due to the other package being bloated)
- Concurrent rotation sensors. This module contains a type that wraps a rotation sensor with the ability to update in the background and not block whilst waiting to be read
- More intuitive leg indexing. In this module, all legs are referred to by their name (a string), not with an index. This makes confusion less likely
