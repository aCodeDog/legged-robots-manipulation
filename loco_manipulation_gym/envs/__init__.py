# SPDX-FileCopyrightText: Copyright (c) 2021 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this
# list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation
# and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its
# contributors may be used to endorse or promote products derived from
# this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
# Copyright (c) 2021 ETH Zurich, Nikita Rudin

from loco_manipulation_gym import LOCO_MANI_GYM_ROOT_DIR, LOCO_MANI_GYM_ENVS_DIR
from .base.legged_robot import LeggedRobot



from loco_manipulation_gym.envs.go2_human.go2_human_config import Go2HumanRoughCfg, Go2HumanRoughCfgPPO
from loco_manipulation_gym.envs.go2_human.go2_human_robot import Go2HumanRobot

from loco_manipulation_gym.envs.go2_arx.go2_arx_config import Go2ArxRoughCfg, Go2ArxRoughCfgPPO
from loco_manipulation_gym.envs.go2_arx.go2_arx_robot import Go2ArxRobot


import os

from loco_manipulation_gym.utils.task_registry import task_registry


task_registry.register( "go2_human", Go2HumanRobot, Go2HumanRoughCfg(), Go2HumanRoughCfgPPO())
task_registry.register( "go2_arx", Go2ArxRobot, Go2ArxRoughCfg(), Go2ArxRoughCfgPPO())




from loco_manipulation_gym.envs.airbot.airbot_robot import Airbot
from loco_manipulation_gym.envs.b2w.b2w_robot import B2w

from loco_manipulation_gym.envs.airbot.airbot_config import AirbotRoughCfg, AirbotRoughCfgPPO
from loco_manipulation_gym.envs.b2w.b2w_config import B2wRoughCfg, B2wRoughCfgPPO



task_registry.register( "b2w", B2w, B2wRoughCfg(), B2wRoughCfgPPO() )
task_registry.register( "airbot", Airbot, AirbotRoughCfg(), AirbotRoughCfgPPO() )