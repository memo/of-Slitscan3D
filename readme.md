Work-in-progress prototype for an upcoming project involving volumetric slitscanning using **kinect** (should it be called surface-scanning?).

Similar to traditional slitscanning (see [flong.com/texts/lists/slit_scan](http://www.flong.com/texts/lists/slit_scan) for more on traditional photographic slitscanning), but instead of working with 2D images + time, this technique uses spatial + temporal data stored in a 4D Space-Time Continuum, and 3 dimensional temporal gradients (i.e. not just slitscanning on the depth/rgb images, but surface-scanning on the animated 3D point cloud).

Bands & blockiness due to relatively low spatial resolution. With more optimisation (e.g. porting to GPU), and on a more powerful processor, resolution can be increased to minimise banding.

Linear temporal gradients (e.g. axis aligned) are much smoother since they need resolution only on one axis (the axis of the gradient). The spherical gradient is quite blocky because it needs resolution on all axes.

Example videos:

[vimeo.com/51461386](https://vimeo.com/51461386)

[vimeo.com/51393499](https://vimeo.com/51393499)

[vimeo.com/51383370](https://vimeo.com/51383370)


Made with [openFrameworks 0072](http://www.openframeworks.cc) and [ofxKinect](http://www.github.com/ofTheo/ofxKinect)

**This code requires a kinect to work (it also runs off webcam, but of course it won't work properly since there is no real 3D data)**