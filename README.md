# `spotpuppy-go` - The quadruped control go module
## What does this module do?
* Makes coding walking algorithms for quadrupedal robots more readable, yet still gives the programmer enough complexity to create complex programs
* Makes it easy to use algorithms between mechanically different robots - when creating a robot you have to provide an IK controller. This can easily be changed to allow lots of different leg configurations.
	* This concept also extends to motor controller drivers and rotation sensors. Both of these are interface types and can be switched out for either pre-build ones, or custom implementations.
* Allows you to work with coordinates, and minimise your interaction with motors/servos and their rotations directly
* Takes care of saving and loading robot configuration to disk - including custom implementations of motor controllers and leg IK drivers
## What robots can run this code?
There are some limitations
* You have to be able to build golang to the platform (I run all of my robots of raspberry pis)
* The robot must have four legs, arranged such that there is one at front right, front left, back right, and back left (similar to boston dynamics spot mini, not a spider like quadruped)

Other than these limitations, you can write motor controllers, rotation sensors, and leg ik drivers for basically anything you can think of, and these components will plug into the module with ease.
## Example Usage Repo
There is a repo with some example code using this module to make a robot _walk_ here `github.com/joshpattman/spotpuppy-go-example`. Further down this readme there are some basic examples if you just want a brief look.
## Example Code
The below code (15 lines!) is all you would need to write to create a robot, then keep its feet at the same points on the floor no matter what rotation the body is at. This particular robot has a `3 servo, one servo in each joint` leg design, a `pca9685` motor driver, and an arduino running arduinodmp code (github.com/joshpattman/TODO).
```go
// Create a quadruped with direct motor IKs and a pca9685 motor controller
q := sp.NewQuadruped(spotpuppy.NewDirectMotorIKGenerator(), pca9685.NewPCAMotorController())

// Create a rotation sensor over a serrial connection to an arduino
mpu := arduinompu.NewArduinoMpu("/dev/ttyUSB0")

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
## Differences to spotpuppy python
You may have noticed that this module shares its name with the `spotpuppy` python library (`github.com/joshpattman/spotpuppy`). This module is inspired by that package, but some less efficient or intuitive areas have been redesigned for maximum simplicity and performance. Some of the changes are:
- Lack of a robot type to extend. You will have to write this type from scratch, however i have found that this actually shortens code and increases readability
- LegIK interface. You can now write custom IK controllers for the legs which allow much easier integration with other leg designs
- Simpler saving/loading. There is now one config file containing everything, with much more concise json
- Faster performance. From some (not very in depth tests), i think this package runs at least 50-100 times faster than the other code (this is not necessarily all pythons fault, and is partly due to the other package being bloated)
- Concurrent rotation sensors. This module contains a type that wraps a rotation sensor with the ability to update in the background and not block whilst waiting to be read
- More intuitive leg indexing. In this module, all legs are referred to by their name (a string), not with an index. This makes confusion less likely
