Once the server module is loaded onto the ABB robot controller, and started (in automatic mode, ABB Menu Button > Production Window, "PP to Main", press physical start button) any computer on the network can very simply interact with the robot via the Python Robot object. It is totally platform independent, working with only the Python Standard Library. 

### Safety
Before moving the robot, you may want to turn the speed down to a low percentage of set speed, do this by clicking the menu button on the lower right of the teach pendant, 25% (or lower)

Keep clear of the robot when the program is running. 

### Lets wave this thing around!


```
$ python
>>> import abb
>>> R = abb.Robot(ip='192.168.125.1')
>>> R.set_cartesian([[1000,0,1000], [0,0,1,0]])
```


And it moves! That command will move it to X= 1000mm, Y=0mm, Z=1000mm. The next four numbers represent [quaternion orientation](http://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation), the important thing to remember being that q = [0,0,1,0] points the tool down and q = [1,0,0,0] points the tool up. 

Usually it is not necessary or desirable to specify the quaternions by hand, but rather rotate around things, check out Christoph Gohlke's [great transformations library](http://www.lfd.uci.edu/~gohlke/code/transformations.py) for this (requires Numpy). 

Other commands include joint moves (where you specify the angles of joints 1-6 in degrees)

```
>>> R.set_joints([0,0,0,0,90,0])
```

And many others. 