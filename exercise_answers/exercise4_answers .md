**Task 1**

you couldn't use relative filepaths???

```
mkdir -p ~/ws/src
cd ~/ws/src
git clone https://github.com/KulunuOS/AUT.700-E4.git . -b humble
cd ~/ws
source /opt/ros/humble/setup.bash
colcon build

cd ~/ws/
source install/setup.bash
ros2 launch walking_actor cam_world.launch.py
```

**Task 2**

**conda activate tracker**

```
mkdir -p ~/miniconda3
wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh -O ~/miniconda3/miniconda.sh
bash ~/miniconda3/miniconda.sh -b -u -p ~/miniconda3
rm ~/miniconda3/miniconda.sh
export PATH=~/miniconda3/bin:$PATH
source ~/.bashrc

source /opt/ros/humble/setup.bash
conda init
conda activate

cd ~/ws/src/Notebooks
conda env create -f environment.yml
conda activate tracker

conda install jupyter
ipython kernel install --user --name=tracker

cd ~/ws/src/Notebooks
conda activate tracker
jupyter-notebook
```



```

```

```
import cv2
import numpy as np
cv2.cvtColor(np.uint8([[[100, 100, 0]]]), cv2.COLOR_BGR2HSV)
```

![image-20251002165730812](/home/jone/snap/typora/106/.config/Typora/typora-user-images/image-20251002165730812.png)

For the test video: enable blue mask

3.3

```
source /opt/ros/humble/setup.bash
cd ~/ws
source install/setup.bash
ros2 launch walking_actor cam_world.launch.py
```

conda activate tracker



