### 类型文件的命名方式

大写字母开头，多个单词命名则每个单词首字母为大写。



### 类型文件的定义 

浮点型： float32
整型：   int32
字符串： string
整型数组： int32[]
浮点型数组： float32[]



### 本包生成的通讯类型
action:  

action类型

```
tutorial_interfaces/action/GoLine 
```


在代码中声明变量：  

```
tutorial_interfaces::action::GoLine
```
使用时需要包含的头文件  

```
#include <tutorial_interfaces/action/go_line.hpp>
```

查看类型的数据内容

```
ros2 interface show tutorial_interfaces/action/GoLine
```



service:

service类型

```
tutorial_interfaces/srv/TurtleCmdMode
```

在代码中声明变量：

```
tutorial_interfaces::srv::TurtleCmdMode
```

使用时需要包含的头文件 

```
#include <tutorial_interfaces/srv/turtle_cmd_mode.hpp>
```

查看类型的数据内容

```
ros2 interface show tutorial_interfaces/srv/TurtleCmdMode
```



topic:

topic类型

```
tutorial_interfaces/msg/Num
```

在代码中声明变量：

```
tutorial_interfaces::msg::Num
```

使用时需要包含的头文件 

```
#include <tutorial_interfaces/msg/num.hpp>
```

查看类型的数据内容

```
ros2 interface show tutorial_interfaces/msg/Num
```

