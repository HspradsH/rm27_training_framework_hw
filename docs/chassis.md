$v_x,v_y$：底盘平动的速度分量（m/s）  
$v_w(\omega)$：底盘自转角速度（rad/s），俯视逆时针为正

辊子沿辊轴方向无法转动、可传力；垂直辊轴方向自由滚动、不传力。控制四个轮子的转动方向来控制摩擦力方向实现全向运动。

### 全向轮  
设：  
$\theta$ 轮组位置与 x 轴正方向的夹角  
$\vec{r}=(R\cos\theta,R\sin\theta,0)$  
$\vec{w}=(0,0,v_w)$  
$\vec{v}_1=(v_x,v_y,0)$  

有：  
$\vec{v}_0=\vec{v}_1+\vec{v}_2$  
$\vec{v}_2=\vec{w}\times \vec{r}$  

则：  
$\vec{v}_2=(-v_w\times R\sin\theta,v_w\times R\cos\theta,0)$  
$\vec{v}_0=(v_x-v_w\times R\sin\theta,v_y+v_w\times R\cos\theta,0)$

$v_0$ 是轮组位置处的速度。  
轮组滚动的方向向量：$\vec{m}=(-\sin\theta,\cos\theta,0)$  
轮子线速度：$v=\vec{v}_0\cdot\vec{m}=(v_x-v_w\times R\sin\theta)\times (-\sin\theta) + (v_y+v_w\times R\cos\theta)\times \cos\theta=-v_x\sin\theta+v_y\cos\theta+v_wR$

轮 0：$\theta=45^\circ$，$v=-\frac{\sqrt 2}{2}v_x+\frac{\sqrt 2}{2}v_y+v_wR$   

轮 1：$\theta=135^\circ$，$v=-\frac{\sqrt 2}{2}v_x-\frac{\sqrt 2}{2}v_y+v_wR$  

轮 2：$\theta=-135^\circ$，$v=\frac{\sqrt 2}{2}v_x-\frac{\sqrt 2}{2}v_y+v_wR$  

轮 3：$\theta=-45^\circ$，$v=\frac{\sqrt 2}{2}v_x+\frac{\sqrt 2}{2}v_y+v_wR$

```text
       x  
       ^  
  轮0  |  轮3  
       |  
y<-----|--------  
       |  
   轮1 |  轮2  
       | 
``` 

### 麦克纳姆轮  
$\vec{v}_0$ 与全向轮一样。  

$\vec{n}$ 方向：  
$\vec{v}_0\cdot \vec{n}=(v\,\vec{m})\cdot \vec{n}$  
$\vec{v}_0\cdot \vec{n}=v (\vec{m}\cdot \vec{n})$  
$\vec{m},\vec{n}$ 都是单位向量  
$\vec{v}_0\cdot \vec{n}=v \cos{45^\circ}$  
$v=\sqrt 2(\vec{v}_0 \cdot\vec {n})$

轮 0：$\vec{n}_0=(\frac{\sqrt 2}{2},-\frac{\sqrt 2}{2})$，$\theta=45^\circ$  
$v=v_x-v_w\times R\sin\theta - (v_y+v_w\times R\cos\theta)=v_x-\frac{\sqrt 2}{2}v_wR - v_y - \frac{\sqrt 2}{2}v_wR=v_x-v_y-\sqrt 2 v_wR$

轮 1：$\vec{n}_1=(\frac{\sqrt 2}{2},\frac{\sqrt 2}{2})$，$\theta=135^\circ$   
$v=v_x+v_y-\sqrt 2 v_wR$

轮 2：$\vec{n}_2=(\frac{\sqrt 2}{2},-\frac{\sqrt 2}{2})$，$\theta=-135^\circ$  
$v=v_x-v_y+\sqrt 2 v_wR$

轮 3：$\vec{n}_3=(\frac{\sqrt 2}{2},\frac{\sqrt 2}{2})$，$\theta=-45^\circ$  
$v=v_x+v_y+\sqrt 2 v_wR$

```text
       x  
       ^  
  轮0/ | \  
    /  |  \轮3  
   /   |   \  
y<-----|--------  
   \   |  /  
  轮1\ | /轮2  
      \|/  
```

### 舵轮  
$\vec{v}_0$ 与前面一致。  
$v=|\vec{v}_0|=\sqrt{(v_x -v_wR\sin\theta)^2 + (v_y+v_wR\cos\theta)^2}$   
$\alpha=\text{atan2}(v_y+v_wR\cos\theta,v_x-v_wR\sin\theta)$（atan2 实现，$y$ 在前，$x$ 在后）