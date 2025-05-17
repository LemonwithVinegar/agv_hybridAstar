float64 start_s
float64 end_s               #混合A*规划出的轨迹，经过分段后每段最后一个点的s
int8    direction       #每一段路径是前进还是倒车，前进是1，后退是-1
int8 lastSegment        #判断规划出的轨迹是否是最后一段 ，LastSegment= 1是最后一段，=0不是
int8 path_id    #每一段路径的id