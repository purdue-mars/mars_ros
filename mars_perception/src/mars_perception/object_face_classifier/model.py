from torch import nn
import torch.nn.functional as F
from mars_perception.object_face_classifier.util import init_weights
import torch


class SimpleClassifierNN(nn.Module):
    def __init__(self, in_dim, num_classes):
        super(SimpleClassifierNN, self).__init__()

        self.in_dim = in_dim

        self.lin1 = nn.Linear(in_dim[0] * in_dim[1], 256)
        self.lin2 = nn.Linear(256, 128)
        self.lin3 = nn.Linear(128, num_classes)

        self.apply(init_weights)

    def forward(self, x: torch.Tensor):
        x = x.flatten(start_dim=1)
        x = F.relu(self.lin1(x))
        x = F.relu(self.lin2(x))
        x = self.lin3(x)
        return x


class SimpleCNN(nn.Module):
    def __init__(self, in_dim, num_classes):
        super(SimpleCNN, self).__init__()
        assert in_dim[1] % 4 == 0 and in_dim[0] % 4 == 0
        self.conv1 = nn.Conv2d(1, 5, kernel_size=3, padding=1)
        self.conv2 = nn.Conv2d(5, 10, kernel_size=3, padding=1)
        self.max_pool = nn.MaxPool2d(kernel_size=2, stride=2)
        lin_input_dim = in_dim[0] // 4 * in_dim[1] // 4 * 10
        self.lin1 = nn.Linear(lin_input_dim, 128)  # two max pools
        self.lin2 = nn.Linear(128, num_classes)  # two max pools
        self.apply(init_weights)

    def forward(self, x: torch.Tensor):  # 48x48x1
        x = F.relu(self.conv1(x))  # 48x48x5
        x = self.max_pool(x)  # 24x24x5
        x = F.relu(self.conv2(x))  # 24x24x10
        x = self.max_pool(x)  # 12x12x10
        x = x.flatten()
        x = F.relu(self.lin1(x))  # 1440
        x = self.lin2(x)  # num_classes
        return x
