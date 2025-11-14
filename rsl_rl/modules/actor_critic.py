# Copyright (c) 2021-2025, ETH Zurich and NVIDIA CORPORATION
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

from __future__ import annotations

import torch
import torch.nn as nn
from torch.distributions import Normal

from rsl_rl.utils import resolve_nn_activation


class ActorCritic(nn.Module):
    is_recurrent = False

    def __init__(
        self,
        num_actor_obs,
        num_critic_obs,
        num_actions,
        actor_hidden_dims=[256, 256, 256],
        critic_hidden_dims=[256, 256, 256],
        activation="relu",
        init_noise_std=1.0,
        noise_std_type: str = "scalar",
        **kwargs,
    ):
        if kwargs:
            print(
                "ActorCritic.__init__ got unexpected arguments, which will be ignored: "
                + str([key for key in kwargs.keys()])
            )
        super().__init__()
        activation = resolve_nn_activation(activation)

        mlp_input_dim_a = num_actor_obs
        mlp_input_dim_c = num_critic_obs
        # Policy
        actor_layers = []
        actor_layers.append(nn.Linear(mlp_input_dim_a, actor_hidden_dims[0]))
        actor_layers.append(activation)
        for layer_index in range(len(actor_hidden_dims)):
            if layer_index == len(actor_hidden_dims) - 1:
                actor_layers.append(nn.Linear(actor_hidden_dims[layer_index], num_actions))
            else:
                actor_layers.append(nn.Linear(actor_hidden_dims[layer_index], actor_hidden_dims[layer_index + 1]))
                actor_layers.append(activation)
        self.actor = nn.Sequential(*actor_layers)

        # policy new
        print("num_actor_obs :",num_actor_obs)
        print("num_critic_obs :",num_critic_obs)
        channels, height, width = [3,128,128]  # CHW 格式
        #########################################################################卷积输入 actor
        # ac_layers = []
        # conv_cfg = [
        #     (32, 8, 4, 0),
        #     (64, 4, 2, 0),
        #     (64, 3, 1, 0)
        # ]
        # in_channels = channels
        # for out_channels, kernel_size, stride, padding in conv_cfg:
        #     ac_layers.append(nn.Conv2d(in_channels, out_channels, kernel_size, stride, padding))
        #     ac_layers.append(nn.ReLU())
        #     in_channels = out_channels
        # ac_layers.append(nn.Flatten(start_dim=1))
        # #网络修改
        # # ac_layers.append(nn.Linear(9216, num_actions))
        # ac_layers.append(nn.Linear(20160, num_actions))
        # self.actor = nn.Sequential(*ac_layers)
        ########################################################################################
        # value new
        # va_layers = []
        # in_channels = channels
        # for out_channels, kernel_size, stride, padding in conv_cfg:
        #     va_layers.append(nn.Conv2d(in_channels, out_channels, kernel_size, stride, padding))
        #     va_layers.append(nn.ReLU())
        #     in_channels = out_channels
        # va_layers.append(nn.Flatten(start_dim=1))
        # va_layers.append(nn.Linear(20160, 1))
        # self.critic = nn.Sequential(*va_layers)
        # # print("self.critic : ",self.critic)
        #######################################################################################
        critic_layers = []
        critic_layers.append(nn.Linear(mlp_input_dim_c, critic_hidden_dims[0]))
        critic_layers.append(activation)
        for layer_index in range(len(critic_hidden_dims)):
            if layer_index == len(critic_hidden_dims) - 1:
                critic_layers.append(nn.Linear(critic_hidden_dims[layer_index], 1))
            else:
                critic_layers.append(nn.Linear(critic_hidden_dims[layer_index], critic_hidden_dims[layer_index + 1]))
                critic_layers.append(activation)
        self.critic = nn.Sequential(*critic_layers)


        print(f"AAActor MLP: {self.actor}")
        print(f"Critic MLP: {self.critic}")

        # Action noise
        self.noise_std_type = noise_std_type
        if self.noise_std_type == "scalar":
            self.std = nn.Parameter(init_noise_std * torch.ones(num_actions))
        elif self.noise_std_type == "log":
            self.log_std = nn.Parameter(torch.log(init_noise_std * torch.ones(num_actions)))
        else:
            raise ValueError(f"Unknown standard deviation type: {self.noise_std_type}. Should be 'scalar' or 'log'")

        # Action distribution (populated in update_distribution)
        self.distribution = None
        # disable args validation for speedup
        Normal.set_default_validate_args(False)

    @staticmethod
    # not used at the moment
    def init_weights(sequential, scales):
        [
            torch.nn.init.orthogonal_(module.weight, gain=scales[idx])
            for idx, module in enumerate(mod for mod in sequential if isinstance(mod, nn.Linear))
        ]

    def reset(self, dones=None):
        pass

    def forward(self):
        raise NotImplementedError

    @property
    def action_mean(self):
        return self.distribution.mean

    @property
    def action_std(self):
        return self.distribution.stddev

    @property
    def entropy(self):
        return self.distribution.entropy().sum(dim=-1)

    def update_distribution(self, observations):
        
        if observations.dim() == 4 and observations.shape[-1] == 3:
            observations = observations.permute(0, 3, 1, 2).contiguous()
        # print("observations : ",observations.shape)
        # compute mean
        mean = self.actor(observations)
        # print("mean : ",mean)
        # compute standard deviation
        if self.noise_std_type == "scalar":
            # print("self.std : ",self.std)
            std = self.std.expand_as(mean)
        elif self.noise_std_type == "log":
            # print("self.log_std : ",self.log_std)
            std = torch.exp(self.log_std).expand_as(mean)
        else:
            raise ValueError(f"Unknown standard deviation type: {self.noise_std_type}. Should be 'scalar' or 'log'")
        # create distribution
        std = torch.clamp(std, min=1e-6)

        self.distribution = Normal(mean, std)

    def act(self, observations, **kwargs):
        self.update_distribution(observations)
        return self.distribution.sample()

    def get_actions_log_prob(self, actions):
        return self.distribution.log_prob(actions).sum(dim=-1)

    def act_inference(self, observations):
        actions_mean = self.actor(observations)
        return actions_mean

    def evaluate(self, critic_observations, **kwargs):
        if critic_observations.dim() == 4 and critic_observations.shape[-1] == 3:
            critic_observations = critic_observations.permute(0, 3, 1, 2).contiguous()
        value = self.critic(critic_observations)
        return value

    def load_state_dict(self, state_dict, strict=True):
        """Load the parameters of the actor-critic model.

        Args:
            state_dict (dict): State dictionary of the model.
            strict (bool): Whether to strictly enforce that the keys in state_dict match the keys returned by this
                           module's state_dict() function.

        Returns:
            bool: Whether this training resumes a previous training. This flag is used by the `load()` function of
                  `OnPolicyRunner` to determine how to load further parameters (relevant for, e.g., distillation).
        """

        super().load_state_dict(state_dict, strict=strict)
        return True
    
import torch
import torch.nn as nn

class CustomCNN(nn.Module):
    def __init__(self, input_shape, activation='relu'):
        super().__init__()
        channels, height, width = input_shape  # CHW 格式

        layers = []
        conv_cfg = [
            (32, 8, 4, 0),
            (64, 4, 2, 0),
            (64, 3, 1, 0)
        ]
        in_channels = channels
        for out_channels, kernel_size, stride, padding in conv_cfg:
            layers.append(nn.Conv2d(in_channels, out_channels, kernel_size, stride, padding))
            layers.append(nn.ReLU() if activation == 'relu' else nn.Identity())
            in_channels = out_channels

        self.conv = nn.Sequential(*layers)

        # 计算 flatten 后维度
        with torch.no_grad():
            dummy_input = torch.zeros(1, channels, height, width)
            self.flatten_dim = self.conv(dummy_input).view(1, -1).shape[1]

        self.flatten = nn.Flatten()
        self.output_dim = self.flatten_dim

    def forward(self, x):
        x = self.conv(x)
        x = self.flatten(x)
        return x
