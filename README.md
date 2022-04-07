# spotpuppy-go
This is a rewrite of the spotpuppy python library (github.com/joshpattman/spotpuppy). I chose to try and write everything again in go as it is faster, typed (easier to find some bugs), and I prefer the syntax.
As with spotpuppy, there is an example repo at github.com/joshpattman/spotpuppy-go-example
# Differences to spotpuppy python
This module is more cut back than the python package. Some changes are
- Lack of a robot type to extend. You will have to write this type from scratch, however i have found that this actually shortens code and increases readability
- LegIK interface. You can now write custom IK controllers for the legs which allow much easier integration with other leg designs
- Simpler saving/loading. There is now one config file containing everything, with much more conscise json
- Faster performance. From some (not very in depth tests), i think this pacakge runs at least 50-100 times faster than the other code (this is not nescisarily all pythons fault, and is partly due to the other package being bloated)
- Concurrent rotation sensors. This module contains a type that wraps a rotation sensor with the ability to update in the background and not block whilst waiting to be read
- More intuitive leg indexing. In this module, all legs are reffered to by their name (a string), not with an index. This makes confusion less likely
