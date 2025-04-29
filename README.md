#### 1.1.1 Setup Environment



- Ubuntu 22.04

- ROS: [Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)

- Install [small_icp](https://github.com/koide3/small_gicp):

  ```
  sudo apt install -y libeigen3-dev libomp-dev
  
  git clone https://github.com/koide3/small_gicp.git
  cd small_gicp
  mkdir build && cd build
  cmake .. -DCMAKE_BUILD_TYPE=Release && make -j
  sudo make install
  ```

  

#### 1.1.2 Create Workspace



```
mkdir -p ~/quadruped_lidar_relocalization_ws
cd ~/quadruped_lidar_relocalization_ws
```



```
git clone --recursive https://github.com/hyyyyyyz/quadruped_hurricane2025_relocalization.git src/quadruped_hurricane2025_relocalization
```

