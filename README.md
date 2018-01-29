# Car-Trailer System Controller
Python project to display and control a trailer going backwards

## Authors
* **Alfredo Hernández**, aldomann.designs@gmail.com
* **David Massip**, david26694@gmail.com
* **Martí Municoy**, mail@martimunicoy.com
* **Jan-Hendrik Niemann**, janhendrik.niemann@e-campus.uab.cat

## Problem description
Backing up a car can sometimes be a stressful experience.
When you have something attached to your car, it gets even more nerve-racking.
The problem is that the motion control while moving forward is stable, while reversing the motion is unstable.

This script uses a geometric model to predict the behavior of the trailer according to the direction we are driving to.
Then, as it knows where the trailer is going to be the next iteration step, it tries to make it follow a desired path.
To tackle this last issue, we designed a controller for the system.
It uses the same mathematical model to modify dynamically the behaviour of the system to follow a particular trajectory.

## Package contents
* _\_\_init\_\_.py_ : defines this Python package
* _simulate.py_ : script to straightforwardly run a simulation
* _controller.py_ : script that handles the system's controller
* _animation.py_ : script that handles the display objects and sets the animation
* _util.py_ : script with general useful functions
* _constants.py_ : definition of constants and default parameters for the whole package

## Prerequisites
This script have been developed and tested with the following Python libraries:
* Python 2.7
* Numpy 1.13.1
* Matplotlib 2.0.2

To save the animation as a mp4 video file, the following Python library is also required:
* ffmpeg 3.4.1

No other Python versions nor library versions have been tested so far.

## Instructions
To run the simulation script, type:
```
./simulate.exe
````

This script allows the user to easily change the parameters of the system.
To do so, you can create an input file with some custom parameters and upload them to the program by using the following command-line argument:
```
./simulate.exe -f path_to_input_file/file.inp
```
You can find an example of an appropriate input file in _inputs/example.inp_


## References

* [1] Nilsson, J., Abraham, S. (2013). Trailer Parking Assist (TPA) (Doctoral dissertation). Chalmers University of Technology

## License
This project is licensed under the GNU License - see the [LICENSE](LICENSE) file for details
