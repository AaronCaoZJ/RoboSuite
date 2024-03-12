from robosuite.models import MujocoWorldBase

world = MujocoWorldBase()

# Step 2: Creating the robot. The class housing the xml of a robot can be created as follows.

from robosuite.models.robots import Panda

mujoco_robot = Panda()

# from robosuite import load_controller_config
# config = load_controller_config(default_controller="OSC_POSE")
# mujoco_robot.default_controller_config = config

# We can add a gripper to the robot by creating a gripper instance and calling the add_gripper method on a robot.

from robosuite.models.grippers import gripper_factory

gripper = gripper_factory('PandaGripper')
mujoco_robot.add_gripper(gripper)

# To add the robot to the world, we place the robot on to a desired position and merge it into the world

mujoco_robot.set_base_xpos([0, 0, 0])
world.merge(mujoco_robot)

# Step 3: Creating the table. We can initialize the TableArena instance that creates a table and the floorplane

from robosuite.models.arenas import TableArena

mujoco_arena = TableArena()
mujoco_arena.set_origin([0.8, 0, 0])
world.merge(mujoco_arena)

# Step 4: Adding the object. For details of MujocoObject, refer to the documentation about MujocoObject, we can create a ball and add it to the world.

from robosuite.models.objects import BallObject
from robosuite.utils.mjcf_utils import new_joint

sphere = BallObject(
    name="sphere",
    size=[0.04],
    rgba=[0, 0.5, 0.5, 1]).get_obj()
sphere.set('pos', '1.0 0 1.0')
world.worldbody.append(sphere)

# Step 5: Running Simulation. Once we have created the object, we can obtain a mujoco_py model by running

m = world.get_model(mode="mujoco")

# This is an MjModel instance that can then be used for simulation. For example,

import mujoco
from mujoco import viewer
import time

d = mujoco.MjData(m)
d.ctrl[:] = [1, 1, 1, 1, 1, 1, 1, 1, 1]

# # 模拟1秒钟
# while data.time < 1:
#     mujoco.mj_step(model, data)

with mujoco.viewer.launch_passive(m, d) as viewer:
  # 30时间步长后关闭viewer
    start = time.time()
    while viewer.is_running() and time.time() - start < 100:
        step_start = time.time()

        # Mj_step可以替换为同样求值的代码
        # mj_step可以替换为同样评估策略并在执行物理之前应用控制信号的代码。
        mujoco.mj_step(m, d)

        # 查看器选项的修改示例：每两秒钟切换一次接触点。
        with viewer.lock():
            viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = int(d.time % 2)

        # 获取物理状态的更改，应用扰动，从GUI更新选项。
        viewer.sync()

        # # 粗略的计时，相对于挂钟会有漂移。
        # time_until_next_step = m.opt.timestep - (time.time() - step_start)
        # if time_until_next_step > 0:
        #     time.sleep(time_until_next_step)
