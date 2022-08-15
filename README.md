# 1. Goal of this project
This Project is to make Visual SLAM with KITTI Data.  
It is based on ProSLAM.  
However, it is based on the theoretical part, not the code. So the implementation is completely different.  
[ProSLAM Github](https://github.com/NamDinhRobotics/proSLAM)  


# 1. Requirements
For easy installation, I made a dockerfile.  
For Thirdparty, I wrote a script file "build.sh".  
If you run the dockerfile, "build.sh" would be started automatically.  

```bash
docker build --no-cache --progress=tty --force-rm -f build.dockerfile -t ${ImageName}:base .

docker run -it --net=host -v /tmp/.X11-unix:/tmp/.X11-unix:ro -v ${local}:${docker} -e DISPLAY=unix$DISPLAY --privileged --name "${ContainerName}" ${ImageName}:base
```

# 2. Run
I made this project with KITTI Data.  
So this needs argument which is path of KITTI Image for Stereo Camera.  
Below is a example.

```bash
cd build
rm -rf *
cmake -G Ninja ..
ninja
./vslam ${somewhere}/sequences/00
```

# 3. Features
## 1. Corner Detection 
I use not ORB but AKAZE or SIFT. Because this is not for real time usage, but for accurate practice.   

## 2. Stereo Camera
I use KITTI's Stereo Image. This makes easy to triangulate 2D Points.  


# 4. Limitations
## 1. Pose estimation
I estimated relative pose using "findEssentialMat()" in OpenCV. 
It is fine in the straight. But in the corner, the error is getting bigger and bigger rapidly. So it is important to optimize pose error.  

## 2. Optimization
For optimization, I use "Ceres-Solver".  
I convert Rotation Matrix to Rodrigues and pass Rodrigues/translation to solver.  
In operator, using "ceres::AngleAxisToRotationMatrix()", I recover Rotation Matrix.  
Under the pinhole camera model, 3D points are projected on current Keyframe and errors are optimized.  
However, the result is weired. Summary of Optimization reports "NO_CONVERGENCE".  
I don't know the exact cause. 

## 3. Map Generation
So this program cannot generate "Global Map" and visualize it. Continuous improvement is needed.

