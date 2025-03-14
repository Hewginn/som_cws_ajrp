# `som_cws_ajrp` package
ROS 2 C++ package.  [![Static Badge](https://img.shields.io/badge/ROS_2-Humble-34aec5)](https://docs.ros.org/en/humble/)
A package 3 nodeból áll. Probléma: Koordináták leggyorsabb bejárása. Az első node /coordinate_publisher véletlen
3 dimenziós koordinátákat publikál egy std_msgs::int32multiarray("coordinates") topicba. Ezt fogadja a /distance_calculator
node, ami kiszámítja a távolságokat a pontok között és továbbítja egy std_msgs::float32multiarray("distances") topicon keresztül.
Végül egy /tsp_solver nevezetű node meg keresi egy Traveling Sales Man algoritmussal a pontok legkönnyebb bejárását,
úgy hogy a vissza utat már nem kell számolni. A pontok sorrendjét a kezdő ponton kívül publikálja egy
std_msgs::int32multiarray("best_order") topicba. Megvalósítás ROS2 Humble alatt.
## Packages and build

It is assumed that the workspace is `~/ros2_ws/`.

### Clone the packages
``` r
cd ~/ros2_ws/src
```
``` r
git clone https://github.com/Hewginn/som_cws_ajrp
```

### Build ROS 2 packages
``` r
cd ~/ros2_ws
```
``` r
colcon build --packages-select som_cws_ajrp --symlink-install
```

<details>
<summary> Don't forget to source before ROS commands.</summary>

``` bash
source ~/ros2_ws/install/setup.bash
```
</details>

``` r
ros2 launch som_cws_ajrp launch_file.launch.py
```
