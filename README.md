# ROS2 레인팔로워 노드

![주행 영상 움짤](https://files.catbox.moe/pe5rfi.webp)

https://github.com/jhoon6/LaneFollower 의 ROS2 노드화

`
ros2 run lanefollower_ros2 pub
`

`
ros2 run lanefollower_ros2 sub
`

pub 실행은 /image/compressed 에서 영상을 받아 엑추에이터 속도를 계산하고 속도 값 토픽을 발행합니다.

sub 실행은 속도 값 토픽을 구독하여 다이나믹셀 엑추에이터에 속도를 전달합니다.

## 실제 주행 영상
- https://www.youtube.com/watch?v=kd0MUSpDgi0
- https://www.youtube.com/watch?v=IZNimrR0728
