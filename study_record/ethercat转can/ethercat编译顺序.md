# 编译顺序

```bash
catkin build any_node 
catkin build ros_ecat_msgs
catkin build ctrl_common
catkin build rc_ecat_master
catkin build radio_hw
catkin build any_node  ros_ecat_msgs  ctrl_common  rc_ecat_master
```

注释掉#include <asm/termbits.h>中的

```cpp
// struct termios {
//     tcflag_t c_iflag;        /* input mode flags */
//     tcflag_t c_oflag;        /* output mode flags */
//     tcflag_t c_cflag;        /* control mode flags */
//     tcflag_t c_lflag;        /* local mode flags */
//     cc_t c_line;            /* line discipline */
//     cc_t c_cc[NCCS];        /* control characters */
// };
```
