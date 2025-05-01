"""

                                                                                  _(\_/)
                                                                                ,((((^`\
                                                                                ((((  (6 \
                                                                              ,((((( ,    \
                                                          ,,,_              ,(((((  /"._  ,`,
                                                          ((((\\ ,...       ,((((   /    `-.-'
                                                          )))  ;'    `"'"'""((((   (      
                                                        (((  /            (((      \
                                                          )) |                      |
                                                        ((  |        .       '     |
                                                        ))  \     _ '      `t   ,.')
                                                        (   |   y;- -,-""'"-.\   \/
                                                        )   / ./  ) /         `\  \
                                                            |./   ( (           / /'
                                                            ||     \\          //'|
                                                            ||      \\       _//'||
                                                            ||       ))     |_/  ||
                                                            \_\     |_/          ||
                                                            `'"                  \_\
                                                                                `'"
"""
# 新疆大学仿生四足机器人全场定位+重定位方案2025总项目仓库

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

#### 1.1.3 Build



```
rosdep install -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
```



```
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```



> Note
>
> 推荐使用 --symlink-install 选项来构建你的工作空间，因为 quadruped_hurricane2025_relocalization 广泛使用了 launch.py 文件和 YAML 文件。这个构建参数会为那些非编译的源文件使用符号链接，这意味着当你调整参数文件时，不需要反复重建，只需要重新启动即可。


